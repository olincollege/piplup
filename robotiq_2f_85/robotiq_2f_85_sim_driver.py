from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from .robotiq_2f_85_constants import *
import numpy as np


class Sim2f85Driver(LeafSystem):
    def __init__(self, plant: MultibodyPlant):
        LeafSystem.__init__(self)

        self.plant = plant
        self.context = self.plant.CreateDefaultContext()

        # Declare input ports
        self.position_port = self.DeclareVectorInputPort("command", BasicVector(1))
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
