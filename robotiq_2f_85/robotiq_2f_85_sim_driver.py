from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from .robotiq_2f_85_constants import *
import numpy as np

# Sim Robotiq 2f-85 Driver
# class Sim2f85Driver(Diagram):
#     def __init__(self, controller_plant: MultibodyPlant):
#         Diagram.__init__(self)
#         builder = DiagramBuilder()
#         num_positions = controller_plant.num_positions()
#         state_demux: Demultiplexer = builder.AddNamedSystem(
#             "demultiplexer", Demultiplexer(2 * num_positions, num_positions)
#         )
#         builder.ExportInput(state_demux.get_input_port(), "state")
#         contact_forces: FirstOrderLowPassFilter = builder.AddNamedSystem(
#             "low_pass_filter", FirstOrderLowPassFilter(0.01, num_positions)
#         )
#         builder.ExportInput(
#             contact_forces.get_input_port(), "generalized_contact_forces"
#         )

#         inverse_dynamics: InverseDynamicsController = builder.AddNamedSystem(
#             "inverse_dynamics_controller",
#             InverseDynamicsController(
#                 controller_plant,
#                 kp=[100] * num_positions,
#                 ki=[1] * num_positions,
#                 kd=[20] * num_positions,
#                 has_reference_acceleration=False,
#             ),
#         )
#         interpolator: StateInterpolatorWithDiscreteDerivative = builder.AddNamedSystem(
#             "velocity_interpolator",
#             StateInterpolatorWithDiscreteDerivative(
#                 num_positions,
#                 0.0005,
#                 suppress_initial_transient=True,
#             ),
#         )
#         builder.ExportInput(interpolator.get_input_port(), "position")
#         builder.ConnectInput("state", inverse_dynamics.GetInputPort("estimated_state"))
#         builder.Connect(
#             interpolator.GetOutputPort("state"),
#             inverse_dynamics.GetInputPort("desired_state"),
#         )

#         builder.ExportOutput(inverse_dynamics.get_output_port(), "actuation")
#         pos_pass = builder.AddNamedSystem(
#             "position_pass_through", PassThrough(num_positions)
#         )
#         builder.ConnectInput("position", pos_pass.get_input_port())
#         builder.ExportOutput(pos_pass.get_output_port(), "position_commanded")

#         builder.ExportOutput(state_demux.get_output_port(0), "position_measured")
#         builder.ExportOutput(state_demux.get_output_port(1), "velocity_estimated")
#         state_pass = builder.AddNamedSystem(
#             "state_pass_through", PassThrough(2 * num_positions)
#         )
#         builder.ConnectInput("state", state_pass.get_input_port())
#         builder.ExportOutput(state_pass.get_output_port(), "state_estimated")
#         builder.ExportOutput(inverse_dynamics.get_output_port(), "torque_commanded")
#         # TODO torque_measured should come from the sim_plant not the controller
#         builder.ExportOutput(inverse_dynamics.get_output_port(), "torque_measured")
#         builder.ExportOutput(contact_forces.get_output_port(), "torque_external")

#         builder.BuildInto(self)


class Sim2f85Driver(LeafSystem):
    def __init__(self, plant:MultibodyPlant):
        LeafSystem.__init__(self)

        self.plant = plant
        self.context = self.plant.CreateDefaultContext()

        # Declare input ports
        self.position_port = self.DeclareVectorInputPort("position", BasicVector(1))
        # self.target_type_port = self.DeclareAbstractInputPort(
        #     "gripper_target_type", AbstractValue.Make(GripperTarget.kVelocity)
        # )

        self.state_port = self.DeclareVectorInputPort(
            "state", BasicVector(self.plant.num_multibody_states())
        )

        # Declare output ports
        self.DeclareVectorOutputPort(
            "applied_gripper_torque", BasicVector(2), self.CalcGripperTorque
        )
        self.DeclareVectorOutputPort(
            "position_measured",
            BasicVector(1),
            self.CalcGripperPosition,
            {self.time_ticket()},
        )
        self.DeclareVectorOutputPort(
            "velocity_measured",
            BasicVector(1),
            self.CalcGripperVelocity,
            {self.time_ticket()},
        )

    def ComputePosition(self, state):
        """
        Compute the gripper position from state data.
        This is especially useful for the 2F-85 gripper, since the
        state does not map neatly to the finger positions.
        """
        # For the more complex 2F-85 gripper, we need to do some kinematics
        # calculations to figure out the gripper position
        self.plant.SetPositionsAndVelocities(self.context, state)

        right_finger = self.plant.GetFrameByName("right_inner_finger_pad")
        left_finger = self.plant.GetFrameByName("left_inner_finger_pad")
        base = self.plant.GetFrameByName("robotiq_arg2f_base_link")

        X_lf = self.plant.CalcRelativeTransform(self.context, left_finger, base)
        X_rf = self.plant.CalcRelativeTransform(self.context, right_finger, base)

        lf_pos = -X_lf.translation()[1]
        rf_pos = -X_rf.translation()[1]

        finger_position = np.array([lf_pos, rf_pos])

        return finger_position

    def ComputeVelocity(self, state):
        """
        Compute the gripper velocity from state data.
        This is especially useful for the 2F-85 gripper, since the
        state does not map neatly to the finger positions.
        """
        # For the more complex 2F-85 gripper, we need to do some kinematics
        self.plant.SetPositionsAndVelocities(self.context, state)
        v = state[-self.plant.num_velocities() :]

        right_finger = self.plant.GetFrameByName("right_inner_finger_pad")
        left_finger = self.plant.GetFrameByName("left_inner_finger_pad")
        base = self.plant.GetFrameByName("robotiq_arg2f_base_link")

        J_lf = self.plant.CalcJacobianTranslationalVelocity(
            self.context,
            JacobianWrtVariable.kV,
            left_finger,
            np.zeros(3),
            base,
            base,
        )
        J_rf = self.plant.CalcJacobianTranslationalVelocity(
            self.context,
            JacobianWrtVariable.kV,
            right_finger,
            np.zeros(3),
            base,
            base,
        )

        lf_vel = -(J_lf @ v)[1]
        rf_vel = (J_rf @ v)[1]

        finger_velocity = np.array([lf_vel, rf_vel])

        return finger_velocity

    def CalcGripperPosition(self, context, output):
        state = self.state_port.Eval(context)

        width = 0.06

        # Send a single number to match the hardware
        both_finger_positions = self.ComputePosition(state)
        net_position = 1 / width * np.mean(both_finger_positions)

        output.SetFromVector([net_position])

    def CalcGripperVelocity(self, context, output):
        state = self.state_port.Eval(context)

        width = 0.06

        # Send a single number to match the hardware
        both_finger_velocity = self.ComputeVelocity(state)
        net_velocity = 1 / width * np.mean(both_finger_velocity)

        output.SetFromVector([net_velocity])

    def CalcGripperTorque(self, context, output):
        state = self.state_port.Eval(context)
        target = self.position_port.Eval(context)

        finger_position = self.ComputePosition(state)
        finger_velocity = self.ComputeVelocity(state)

        width = 0.06
        Kp = 10 * np.eye(2)
        Kd = 2 * np.sqrt(0.01 * Kp)

        target = width - width * target * np.ones(2)
        target_finger_position = target
        target_finger_velocity = np.zeros(2)

        # Determine applied torques with PD controller
        position_err = target_finger_position - finger_position
        velocity_err = target_finger_velocity - finger_velocity
        tau = -Kp @ (position_err) - Kd @ (velocity_err)

        output.SetFromVector(tau)