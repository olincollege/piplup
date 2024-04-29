from pydrake.all import *
from oculus_reader.reader import OculusReader
from copy import copy
import time


class QuestTwistTeleopController(LeafSystem):
    def __init__(self, meshcat: Meshcat, hand_model_name: str):
        super().__init__()
        self._meshcat = meshcat
        self._time_step = 0.0001
        self.grip_speed = 0.8
        self.hand_model_name = hand_model_name

        self.oculus_reader = OculusReader()
        self.movement_freeze_pose = None
        self.ee_freeze_pose = None

        self.X_WE_desired_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(RigidTransform())
        )

        if hand_model_name == "2f_85":
            self.gripper_cmd_state_idx = self.DeclareDiscreteState(1)
        elif hand_model_name == "epick":
            self.gripper_cmd_state_idx = self.DeclareAbstractState(Value(False))
        else:
            raise RuntimeError("Invalid hand model name")

        self.buttons_state_idx = self.DeclareAbstractState(AbstractValue.Make(dict()))

        self.robot_pose_port = self.DeclareVectorInputPort("pose", 6)
        self.DeclareVectorOutputPort("V_WE_desired", 6, self.CalcTwist)
        self.DeclareStateOutputPort("gripper_command", self.gripper_cmd_state_idx)
        self.DeclareStateOutputPort("buttons", self.buttons_state_idx)
        self.DeclarePeriodicDiscreteUpdateEvent(self._time_step, 0, self.Integrate)
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)

    def Initialize(self, context: Context, discrete_state: DiscreteValues):
        pose = self.robot_pose_port.Eval(context)
        ee_pose = RigidTransform(RollPitchYaw(pose[:3]).ToQuaternion(), pose[3:])
        context.SetAbstractState(self.X_WE_desired_state_idx, ee_pose)

    def Integrate(self, context: Context, discrete_state: DiscreteValues):
        X_WE_desired: RigidTransform = context.get_abstract_state(
            self.X_WE_desired_state_idx
        ).get_mutable_value()

        transforms, buttons = self.oculus_reader.get_transformations_and_buttons()
        enable = False
        reset = False
        pose_delta = RigidTransform()
        if "A" in buttons:
            enable = buttons["A"]
            reset = buttons["B"]
        if "r" in transforms:
            controller_pose = RigidTransform(Isometry3(transforms["r"]))

        if enable and not self.movement_freeze_pose:
            self.movement_freeze_pose = controller_pose
        elif enable:
            pose_delta = self.movement_freeze_pose.InvertAndCompose(controller_pose)
            position = copy(pose_delta.translation())
            position[1] = -position[1]
            position[2] = -position[2]
            # pose_delta.set_translation(np.zeros_like(position))
            pose_delta.set_translation(position)
            rot = RollPitchYaw(pose_delta.rotation())
            rpy = rot.vector()
            rpy[1] = -rpy[1]
            rpy[2] = -rpy[2]
            pose_delta.set_rotation(RollPitchYaw(rpy))
        else:
            self.movement_freeze_pose = None
            self.ee_freeze_pose = copy(X_WE_desired)
            reset = True

        if buttons:
            context.SetAbstractState(self.buttons_state_idx, copy(buttons))
            if self.hand_model_name == "2f_85":
                clipped_vel = np.array(buttons["rightTrig"]) - np.array(
                    buttons["rightGrip"]
                )
                discrete_state.set_value(
                    self.gripper_cmd_state_idx,
                    clipped_vel * (-self.grip_speed),
                )
            elif self.hand_model_name == "epick":
                discrete_state.set_value(
                    self.gripper_cmd_state_idx, buttons["rightBumper"]
                )
        if reset:
            pose = self.robot_pose_port.Eval(context)
            X_WE = RigidTransform(RollPitchYaw(pose[:3]).ToQuaternion(), pose[3:])
            X_WE_desired.set_rotation(X_WE.rotation())
            X_WE_desired.set_translation(X_WE.translation())
        else:
            X_WE_desired.set_rotation(
                pose_delta.multiply(self.ee_freeze_pose).rotation()
            )
            X_WE_desired.set_translation(
                self.ee_freeze_pose.translation() + pose_delta.translation()
            )
        viz_color = Rgba(0, 0.5, 0, 0.5) if enable else Rgba(0, 0, 0.5, 0.5)
        self._meshcat.SetObject("ee_sphere", Sphere(0.05), viz_color)
        self._meshcat.SetTransform("/drake/ee_sphere", X_WE_desired)
        self._meshcat.SetTransform("/drake/ee_body", X_WE_desired)

    def CalcTwist(self, context: Context, output: BasicVector):
        twist = np.zeros(6)
        X_WE_desired: RigidTransform = context.get_abstract_state(
            self.X_WE_desired_state_idx
        ).get_value()
        pose = self.robot_pose_port.Eval(context)
        X_WE = RigidTransform(RollPitchYaw(pose[:3]).ToQuaternion(), pose[3:])
        self._meshcat.SetObject("test", Sphere(0.05), Rgba(0.5, 0.5, 0, 0.5))
        self._meshcat.SetTransform("/drake/test", X_WE)
        self._meshcat.SetTransform("/drake/test_body", X_WE)

        error_pose = X_WE.InvertAndCompose(X_WE_desired)
        translation_error = X_WE_desired.translation() - X_WE.translation()
        rpy_error = RollPitchYaw(error_pose.rotation()).vector()
        twist[:3] = 1 * (X_WE.rotation().matrix() @ rpy_error)
        twist[3:] = 1.5 * translation_error
        self.last_translation_error = translation_error
        # self.last_t =
        output.SetFromVector(twist)


class QuestTeleopController(LeafSystem):
    def __init__(
        self, meshcat: Meshcat, controller_plant: MultibodyPlant, hand_model_name: str
    ):
        super().__init__()
        self._meshcat = meshcat
        self._time_step = 0.01
        self.controller_plant = controller_plant
        self.world_frame = controller_plant.world_frame()
        self.ee_frame = controller_plant.GetFrameByName("tool_frame")
        self.plant_context = controller_plant.CreateDefaultContext()

        self.oculus_reader = OculusReader()
        self.movement_freeze_pose = None
        self.ee_freeze_pose = None

        self.X_WE_desired_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(RigidTransform())
        )

        self.gripper_cmd_state_idx = self.DeclareDiscreteState(1)
        self.b_state_idx = self.DeclareAbstractState(AbstractValue.Make(False))

        self.robot_state_port = self.DeclareVectorInputPort(
            "ee_pose", 6
        )
        self.DeclareStateOutputPort("X_WE_desired", self.X_WE_desired_state_idx)
        self.DeclareStateOutputPort("gripper_command", self.gripper_cmd_state_idx)
        self.DeclareStateOutputPort("b", self.b_state_idx)
        self.DeclarePeriodicDiscreteUpdateEvent(self._time_step, 0, self.Integrate)
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)

    def Initialize(self, context: Context, discrete_state: DiscreteValues):
        robot_state = self.robot_state_port.Eval(context)
        # self.controller_plant.SetPositions(self.plant_context, robot_state[:7])
        # ee_pose: RigidTransform = self.controller_plant.CalcRelativeTransform(
        #     self.plant_context, self.world_frame, self.ee_frame
        # )
        ee_pose = RigidTransform(RollPitchYaw(robot_state[:3]).ToQuaternion(), robot_state[3:])
        context.SetAbstractState(self.X_WE_desired_state_idx, ee_pose)

    def Integrate(self, context: Context, discrete_state: DiscreteValues):
        X_WE_desired: RigidTransform = context.get_abstract_state(
            self.X_WE_desired_state_idx
        ).get_value()

        transforms, buttons = self.oculus_reader.get_transformations_and_buttons()

        enable = False
        pose_delta = RigidTransform()
        if "A" in buttons:
            enable = buttons["A"]
        if "r" in transforms:
            controller_pose = RigidTransform(Isometry3(transforms["r"]))

        if enable and not self.movement_freeze_pose:
            self.movement_freeze_pose = controller_pose
        elif enable:
            # TODO these calcs are a bit wrong
            pose_delta = controller_pose.InvertAndCompose(self.movement_freeze_pose)
            position = copy(pose_delta.translation())
            position[2] = -position[2]
            # pose_delta.set_translation(np.zeros_like(position))
            pose_delta.set_translation(position)
            rot = RollPitchYaw(pose_delta.rotation())
            rpy = rot.vector()
            rpy[2] = -rpy[2]
            pose_delta.set_rotation(RollPitchYaw(rpy))
        else:
            self.movement_freeze_pose = None
            self.ee_freeze_pose = copy(X_WE_desired)

        if "rightTrig" in buttons:
            discrete_state.set_value(
                self.gripper_cmd_state_idx, np.array(buttons["rightTrig"])
            )

        if "B" in buttons:
            discrete_state.set_value(self.b_state_idx, buttons["B"])

        context.SetAbstractState(
            self.X_WE_desired_state_idx, self.ee_freeze_pose.multiply(pose_delta)
        )

        X_WE = X_WE_desired
        viz_color = Rgba(0, 0.5, 0, 0.5) if enable else Rgba(0, 0, 0.5, 0.5)
        self._meshcat.SetObject("ee_sphere", Sphere(0.05), viz_color)

        self._meshcat.SetTransform("/drake/ee_sphere", X_WE)
        self._meshcat.SetTransform("/drake/ee_body", X_WE)
