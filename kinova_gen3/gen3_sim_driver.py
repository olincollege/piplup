from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from .gen3_constants import *
import numpy as np


class TwistIntegrator(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        self.twist_idx = self.DeclareVectorInputPort("twist", 6)
        self.DeclareAbstractOutputPort(
            "pose", AbstractValue.Make(RigidTransform()), self.CalcPose
        )

        self.DeclareContinuousState()

    def CalcPose(self, context: Context, output: RigidTransform):
        self.twist_idx.Eval(context)


class Gen3ControlModeToIndex(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.mode_port_ = self.DeclareAbstractInputPort(
            "control_mode", AbstractValue.Make(Gen3ControlMode.kPose)
        )
        self.DeclareAbstractOutputPort(
            "control_mode",
            lambda: AbstractValue.Make(InputPortIndex(0)),
            self.CalcControlMode,
        )

        self.DeclareAbstractOutputPort(
            "diff_ik_reset", lambda: AbstractValue.Make(False), self.CalcDiffIkReset
        )

    def CalcControlMode(self, context: Context, output: AbstractValue):
        control_mode: Gen3ControlMode = self.mode_port_.Eval(context)
        match control_mode:
            case Gen3ControlMode.kPosition:
                output.set_value(InputPortIndex(1))
            case Gen3ControlMode.kVelocity:
                output.set_value(InputPortIndex(2))
            case Gen3ControlMode.kPose:
                output.set_value(InputPortIndex(3))
            case _:
                raise RuntimeWarning(f"Unsupported Sim Control Mode: {control_mode}")

    def CalcDiffIkReset(self, context: Context, output: AbstractValue):
        control_mode: Gen3ControlMode = self.mode_port_.Eval(context)
        output.set_value(control_mode != Gen3ControlMode.kPose)


class RPYXYZToRigidTransformSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.pose_port_ = self.DeclareVectorInputPort("pose", 7)
        self.DeclareAbstractOutputPort(
            "pose", lambda: AbstractValue.Make(RigidTransform()), self.CalcPose
        )

    def CalcPose(self, context: Context, output: AbstractValue):
        rpy_xyz = self.pose_port_.Eval(context)[:6]
        pose = RigidTransform()
        pose.set_rotation(RollPitchYaw(rpy_xyz[:3]).ToQuaternion())
        pose.set_translation(rpy_xyz[3:])
        output.set_value(pose)


# Sim Gen3 Driver
class SimGen3Driver(Diagram):
    def __init__(self, controller_plant: MultibodyPlant):
        Diagram.__init__(self)
        builder = DiagramBuilder()
        num_positions = controller_plant.num_positions()
        state_demux: Demultiplexer = builder.AddNamedSystem(
            "demultiplexer", Demultiplexer(2 * num_positions, num_positions)
        )
        builder.ExportInput(state_demux.get_input_port(), "state")
        contact_forces: FirstOrderLowPassFilter = builder.AddNamedSystem(
            "low_pass_filter", FirstOrderLowPassFilter(0.01, num_positions)
        )
        builder.ExportInput(
            contact_forces.get_input_port(), "generalized_contact_forces"
        )

        inverse_dynamics: InverseDynamicsController = builder.AddNamedSystem(
            "inverse_dynamics_controller",
            InverseDynamicsController(
                controller_plant,
                kp=[100] * num_positions,
                ki=[1] * num_positions,
                kd=[20] * num_positions,
                has_reference_acceleration=False,
            ),
        )
        interpolator: StateInterpolatorWithDiscreteDerivative = builder.AddNamedSystem(
            "velocity_interpolator",
            StateInterpolatorWithDiscreteDerivative(
                num_positions,
                0.0005,
                suppress_initial_transient=True,
            ),
        )

        params = DifferentialInverseKinematicsParameters(
            controller_plant.num_positions(), controller_plant.num_velocities()
        )
        time_step = 0.005
        params.set_time_step(time_step)
        params.set_end_effector_angular_speed_limit(2)
        params.set_end_effector_translational_velocity_limits([-2, -2, -2], [2, 2, 2])
        params.set_joint_velocity_limits(
            get_gen3_joint_velocity_limits(controller_plant)
        )
        params.set_joint_position_limits(
            get_gen3_joint_position_limits(controller_plant)
        )
        frame_E = controller_plant.GetFrameByName("tool_frame")
        diff_ik: DifferentialInverseKinematicsIntegrator = builder.AddSystem(
            DifferentialInverseKinematicsIntegrator(
                controller_plant, frame_E, time_step, params
            )
        )

        vel_integrator: Integrator = builder.AddNamedSystem(
            "velocity_integrator", Integrator(7)
        )

        port_switch: PortSwitch = builder.AddNamedSystem(
            "control_mode_switch", PortSwitch(7)
        )

        control_mode_to_switch: Gen3ControlModeToIndex = builder.AddNamedSystem(
            "mode_to_switch", Gen3ControlModeToIndex()
        )

        diff_ik_pose: RPYXYZToRigidTransformSystem = builder.AddNamedSystem(
            "diff_ik_pose", RPYXYZToRigidTransformSystem()
        )
        builder.ExportInput(diff_ik_pose.GetInputPort("pose"), "command")
        builder.ConnectInput("command", port_switch.DeclareInputPort("position"))
        builder.ConnectInput("command", vel_integrator.get_input_port())

        builder.ExportInput(control_mode_to_switch.get_input_port(), "control_mode")
        builder.Connect(
            control_mode_to_switch.GetOutputPort("control_mode"),
            port_switch.get_port_selector_input_port(),
        )

        builder.ConnectInput("state", diff_ik.GetInputPort("robot_state"))

        builder.Connect(
            control_mode_to_switch.GetOutputPort("diff_ik_reset"),
            diff_ik.GetInputPort("use_robot_state"),
        )
        builder.Connect(
            diff_ik_pose.get_output_port(), diff_ik.GetInputPort("X_WE_desired")
        )
        builder.Connect(
            vel_integrator.get_output_port(),
            port_switch.DeclareInputPort("velocity"),
        )
        builder.Connect(
            diff_ik.GetOutputPort("joint_positions"),
            port_switch.DeclareInputPort("diff_ik_pose"),
        )

        builder.Connect(port_switch.get_output_port(), interpolator.get_input_port())
        builder.ExportOutput(port_switch.get_output_port(), "position_commanded")

        builder.ConnectInput("state", inverse_dynamics.GetInputPort("estimated_state"))
        builder.Connect(
            interpolator.GetOutputPort("state"),
            inverse_dynamics.GetInputPort("desired_state"),
        )

        builder.ExportOutput(inverse_dynamics.get_output_port(), "actuation")

        builder.ExportOutput(state_demux.get_output_port(0), "position_measured")
        builder.ExportOutput(state_demux.get_output_port(1), "velocity_estimated")
        state_pass = builder.AddNamedSystem(
            "state_pass_through", PassThrough(2 * num_positions)
        )
        builder.ConnectInput("state", state_pass.get_input_port())
        builder.ExportOutput(state_pass.get_output_port(), "state_estimated")
        builder.ExportOutput(inverse_dynamics.get_output_port(), "torque_commanded")
        # TODO torque_measured should come from the sim_plant not the controller (krishna)
        builder.ExportOutput(inverse_dynamics.get_output_port(), "torque_measured")
        builder.ExportOutput(contact_forces.get_output_port(), "torque_external")

        builder.BuildInto(self)
