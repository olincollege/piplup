from pydrake.all import *
from oculus_reader.reader import OculusReader
from copy import copy

class QuestEndeffectorController(LeafSystem):
    def __init__(self, meshcat: Meshcat, controller_plant: MultibodyPlant, hand_model_name: str):
        super().__init__()
        self._meshcat = meshcat
        self._time_step = 0.005
        self.controller_plant = controller_plant
        self.world_frame = controller_plant.world_frame()
        self.ee_frame = controller_plant.GetFrameByName("end_effector_frame")
        self.plant_context = controller_plant.CreateDefaultContext()

        self.oculus_reader = OculusReader()
        self.movement_freeze_pose = None
        self.ee_freeze_pose = None

        self.X_WE_desired_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(RigidTransform())
        )

        self.gripper_cmd_state_idx = self.DeclareDiscreteState(1)

        self.robot_state_port = self.DeclareVectorInputPort(
            "robot_state", controller_plant.num_multibody_states()
        )
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

        transforms, buttons = self.oculus_reader.get_transformations_and_buttons()
        enable = False
        pose_delta = RigidTransform()
        if 'A' in buttons:
            enable = buttons['A']
        if "r" in transforms:
            controller_pose = RigidTransform(Isometry3(transforms["r"]))

        if enable and not self.movement_freeze_pose:
            self.movement_freeze_pose = controller_pose
        elif enable:
            pose_delta = self.movement_freeze_pose.InvertAndCompose(controller_pose)
        else:
            self.movement_freeze_pose = None
            self.ee_freeze_pose = copy(X_WE_desired)

        
        context.SetAbstractState(self.X_WE_desired_state_idx, self.ee_freeze_pose.multiply(pose_delta))
        
        X_WE = X_WE_desired
        viz_color = Rgba(0, 0.5, 0, 0.5) if enable else Rgba(0, 0, 0.5, 0.5)
        self._meshcat.SetObject("ee_sphere", Sphere(0.05), viz_color)

        self._meshcat.SetTransform("/drake/ee_sphere", X_WE)
        self._meshcat.SetTransform("/drake/ee_body", X_WE)
