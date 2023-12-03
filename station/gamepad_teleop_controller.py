from pydrake.all import *
import numpy as np
from pydrake.geometry import Meshcat
from copy import copy

class GamepadTwistTeleopController(LeafSystem):
    def __init__(
        self, meshcat: Meshcat, controller_plant: MultibodyPlant, hand_model_name: str
    ):
        super().__init__()
        self._meshcat = meshcat
        self.linear_speed = 0.1
        self.angular_speed = 0.8
        self.grip_speed = 0.25
        self.hand_model_name = hand_model_name

        self.linear_mode_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(True)
        )  # gamepad mode
        # This can be used to specific arbitrary gripper commands

        self.robot_pose_port = self.DeclareVectorInputPort("pose", 6)
        self.DeclareVectorOutputPort("V_WE_desired", 6, self.OutputTwist)
        self.DeclareVectorOutputPort("gripper_command", 1, self.OutputGripper)

    def OutputTwist(self, context: Context, output: BasicVector):
        pose = self.robot_pose_port.Eval(context)
        X_WE_desired = RigidTransform(RollPitchYaw(pose[:3]).ToQuaternion(),pose[3:])
        linear_mode = context.get_abstract_state(self.linear_mode_state_idx).get_value()
        target_twist = np.zeros(6)

        gamepad = self._meshcat.GetGamepad()
        if not gamepad.index == None:

            def CreateStickDeadzone(x, y):
                stick = np.array([x, y])
                deadzone = 0.3
                m = np.linalg.norm(stick)
                if m < deadzone:
                    return np.array([0, 0])
                over = (m - deadzone) / (1 - deadzone)
                return stick * over / m

            left = CreateStickDeadzone(gamepad.axes[0], -gamepad.axes[1])
            right = CreateStickDeadzone(-gamepad.axes[2], gamepad.axes[3])

            if gamepad.button_values[8]:
                context.SetAbstractState(self.linear_mode_state_idx, True)
            if gamepad.button_values[9]:
                context.SetAbstractState(self.linear_mode_state_idx, False)
            if linear_mode:
                target_twist[3:] = (
                    np.array([left[0], left[1], -right[1]]) * self.linear_speed
                )
            else:
                ee_rot = X_WE_desired.rotation()
                target_twist[:3] = ee_rot.inverse().multiply(
                    np.array(
                        [
                            -left[0],
                            left[1],
                            right[1],
                        ]
                    )
                    * self.angular_speed
                )
        output.SetFromVector(target_twist)
        
    def OutputGripper(self, context: Context, output: BasicVector):
        gamepad = self._meshcat.GetGamepad()
        if self.hand_model_name == "2f_85":
            cmd_vel = np.zeros(1)
            if not gamepad.index == None:
                gripper_close = gamepad.button_values[6] * 3
                gripper_open = gamepad.button_values[7] * 3
                cmd_vel = np.array([(gripper_close - gripper_open) / 2])*self.grip_speed
            output.set_value(cmd_vel)
        elif self.hand_model_name == "epick_2cup":
            pass


class GamepadTeleopController(LeafSystem):
    def __init__(
        self, meshcat: Meshcat, controller_plant: MultibodyPlant, hand_model_name: str
    ):
        super().__init__()
        self._meshcat = meshcat
        self._time_step = 0.005
        self.linear_speed = 0.1
        self.angular_speed = 0.8
        self.controller_plant = controller_plant
        self.world_frame = controller_plant.world_frame()
        self.ee_frame = controller_plant.GetFrameByName("tool_frame")
        self.plant_context = controller_plant.CreateDefaultContext()

        self.X_WE_desired_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(RigidTransform())
        )
        self.linear_mode_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(True)
        )  # gamepad mode
        self.hand_model_name = hand_model_name
        # This can be used to specific arbitrary gripper commands
        if hand_model_name == "2f_85":
            self.gripper_cmd_state_idx = self.DeclareDiscreteState(
                1
            )  # Gripper position
        elif hand_model_name == "epick_2cup":
            self.gripper_cmd_state_idx = self.DeclareDiscreteState(2)  # Suction cmd

        self.robot_state_port = self.DeclareVectorInputPort(
            "robot_state", controller_plant.num_multibody_states()
        )
        # self.robot_pose_port = self.DeclareAbstractInputPort("pose", Value(RigidTransform()))
        # self.robot_pose_port = self.DeclareVectorInputPort("pose", 6)
        self.DeclareStateOutputPort("X_WE_desired", self.X_WE_desired_state_idx)
        self.DeclareStateOutputPort("gripper_command", self.gripper_cmd_state_idx)
        self.DeclarePeriodicDiscreteUpdateEvent(self._time_step, 0, self.Integrate)
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)

    def Initialize(self, context: Context, discrete_state: DiscreteValues):
        robot_state = self.robot_state_port.Eval(context)
        self.controller_plant.SetPositions(self.plant_context, robot_state[:7])
        ee_pose: RigidTransform = self.controller_plant.CalcRelativeTransform(
            self.plant_context, self.world_frame, self.ee_frame
        )
        context.SetAbstractState(self.X_WE_desired_state_idx, ee_pose)

    def Integrate(self, context: Context, discrete_state: DiscreteValues):
        X_WE_desired: RigidTransform = context.get_abstract_state(
            self.X_WE_desired_state_idx
        ).get_value()
        linear_mode = context.get_abstract_state(self.linear_mode_state_idx).get_value()
        target_twist = np.zeros(6)

        gamepad = self._meshcat.GetGamepad()
        if not gamepad.index == None:

            def CreateStickDeadzone(x, y):
                stick = np.array([x, y])
                deadzone = 0.3
                m = np.linalg.norm(stick)
                if m < deadzone:
                    return np.array([0, 0])
                over = (m - deadzone) / (1 - deadzone)
                return stick * over / m

            left = CreateStickDeadzone(gamepad.axes[0], -gamepad.axes[1])
            right = CreateStickDeadzone(-gamepad.axes[2], gamepad.axes[3])

            if gamepad.button_values[8]:
                context.SetAbstractState(self.linear_mode_state_idx, True)
            if gamepad.button_values[9]:
                context.SetAbstractState(self.linear_mode_state_idx, False)
            if linear_mode:
                target_twist[3:] = (
                    np.array([left[0], left[1], -right[1]]) * self.linear_speed
                )
            else:
                ee_rot = X_WE_desired.rotation()
                target_twist[:3] = ee_rot.inverse().multiply(
                    np.array(
                        [
                            -left[0],
                            left[1],
                            right[1],
                        ]
                    )
                    * self.angular_speed
                )

        X_WE_desired.set_translation(
            X_WE_desired.translation() + target_twist[3:] * self._time_step
        )

        R_delta = RotationMatrix(RollPitchYaw(target_twist[:3] * self._time_step))
        X_WE_desired.set_rotation(
            X_WE_desired.rotation().multiply(R_delta).ToQuaternion()
        )

        # TODO this can be better designed to have the command specification passed in as a call back rather than defined here (krishna)
        if self.hand_model_name == "2f_85":
            cmd_pos = np.copy(discrete_state.get_value(self.gripper_cmd_state_idx))
            if not gamepad.index == None:
                gripper_close = gamepad.button_values[6] * 3
                gripper_open = gamepad.button_values[7] * 3
                pos_delta = (
                    np.array([(gripper_close - gripper_open) / 2]) * self._time_step
                )
                cmd_pos = np.clip(cmd_pos + pos_delta, 0, 1)
            discrete_state.set_value(self.gripper_cmd_state_idx, cmd_pos)
        elif self.hand_model_name == "epick_2cup":
            if not gamepad.index == None:
                discrete_state.set_value(
                    self.gripper_cmd_state_idx,
                    np.array([gamepad.button_values[6], gamepad.button_values[7]]),
                )
        context.SetAbstractState(self.X_WE_desired_state_idx, X_WE_desired)

        viz_color = Rgba(0, 0.5, 0, 0.5) if linear_mode else Rgba(0, 0, 0.5, 0.5)
        self._meshcat.SetObject("ee_sphere", Sphere(0.05), viz_color)

        X_WE = copy(X_WE_desired)
        self._meshcat.SetTransform("/drake/ee_sphere", X_WE)
        self._meshcat.SetTransform("/drake/ee_body", X_WE)

        # robot_state = self.robot_state_port.Eval(context)
        # self.controller_plant.SetPositions(self.plant_context, robot_state[:7])
        # ee_pose: RigidTransform = self.controller_plant.CalcRelativeTransform(
        #     self.plant_context, self.world_frame, self.ee_frame
        # )
        # self._meshcat.SetObject("ee_sphere_target", Sphere(0.05), Rgba(0.5, 0, 0, 0.5))
        # self._meshcat.SetTransform("/drake/ee_sphere_target", ee_pose)

        # pose = self.robot_pose_port.Eval(context)
        # test_pose = RigidTransform(RollPitchYaw(pose[:3]).ToQuaternion(),pose[3:])
        # self._meshcat.SetObject("test", Sphere(0.05), Rgba(0.5, 0.5, 0, 0.5))
        # self._meshcat.SetTransform("/drake/test", test_pose)
        # self._meshcat.SetTransform("/drake/test_body", test_pose)
