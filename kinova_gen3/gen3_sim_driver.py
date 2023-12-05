from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from .gen3_constants import *
import numpy as np


# Sim Gen3 Driver
class SimGen3Driver(Diagram):
    def __init__(self, controller_plant: MultibodyPlant, control_mode: Gen3ControlMode):
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

        if control_mode == Gen3ControlMode.kPose:
            params = DifferentialInverseKinematicsParameters(
                controller_plant.num_positions(), controller_plant.num_velocities()
            )
            # q0 = plant.GetPositions(plant.CreateDefaultContext())
            # params.set_nominal_joint_position(q0)
            time_step = 0.005
            params.set_time_step(time_step)
            params.set_end_effector_angular_speed_limit(2)
            params.set_end_effector_translational_velocity_limits(
                [-2, -2, -2], [2, 2, 2]
            )
            # params.set_joint_centering_gain(1 * np.eye(7))
            params.set_joint_velocity_limits(
                get_gen3_joint_velocity_limits(controller_plant)
            )
            params.set_joint_position_limits(
                get_gen3_joint_position_limits(controller_plant)
            )

            frame_E = controller_plant.GetFrameByName(
                "tool_frame"
            )  # TODO should be dynamically set from the config

            diff_ik: DifferentialInverseKinematicsIntegrator = builder.AddSystem(
                DifferentialInverseKinematicsIntegrator(
                    controller_plant, frame_E, time_step, params
                )
            )
            builder.ExportInput(diff_ik.GetInputPort("X_WE_desired"), "pose")
            builder.ConnectInput("state", diff_ik.GetInputPort("robot_state"))
            builder.Connect(
                diff_ik.GetOutputPort("joint_positions"), interpolator.get_input_port()
            )
            builder.ExportOutput(
                diff_ik.GetOutputPort("joint_positions"), "position_commanded"
            )
        elif control_mode == Gen3ControlMode.kPosition:
            builder.ExportInput(interpolator.get_input_port(), "position")
            pos_pass = builder.AddNamedSystem(
                "position_pass_through", PassThrough(num_positions)
            )
            builder.ExportOutput(pos_pass.get_output_port(), "position_commanded")
            builder.ConnectInput("position", pos_pass.get_input_port())
        else:
            raise RuntimeError(f"Unsupported Control Mode: {control_mode}")

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
