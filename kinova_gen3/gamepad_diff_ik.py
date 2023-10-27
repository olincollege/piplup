from pydrake.all import *
import numpy as np
from pydrake.geometry import Meshcat, SceneGraph
from .gen3_constants import (
    get_gen3_joint_velocity_limits,
    get_gen3_joint_position_limits,
)
from copy import copy


# TODO Move to a different package
class GamepadDiffIkController(Diagram):
    def __init__(self, meshcat: Meshcat, controller_plant: MultibodyPlant):
        super().__init__()

        params = DifferentialInverseKinematicsParameters(
            controller_plant.num_positions(), controller_plant.num_velocities()
        )
        # q0 = plant.GetPositions(plant.CreateDefaultContext())
        # params.set_nominal_joint_position(q0)
        time_step = 0.005
        params.set_time_step(time_step)
        params.set_end_effector_angular_speed_limit(2)
        params.set_end_effector_translational_velocity_limits([-2, -2, -2], [2, 2, 2])
        # params.set_joint_centering_gain(1 * np.eye(7))
        params.set_joint_velocity_limits(
            get_gen3_joint_velocity_limits(controller_plant)
        )
        params.set_joint_position_limits(
            get_gen3_joint_position_limits(controller_plant)
        )

        frame_E = controller_plant.GetFrameByName("end_effector_frame")

        builder = DiagramBuilder()
        gamepad: GamepadPoseIntegrator = builder.AddNamedSystem(
            "gamepad", GamepadPoseIntegrator(meshcat, controller_plant)
        )
        diff_ik: DifferentialInverseKinematicsIntegrator = builder.AddSystem(
            DifferentialInverseKinematicsIntegrator(
                controller_plant, frame_E, time_step, params
            )
        )
        builder.Connect(
            gamepad.GetOutputPort("X_WE_desired"), diff_ik.GetInputPort("X_WE_desired")
        )
        builder.ExportInput(
            diff_ik.GetInputPort("robot_state"),
            "gen3.state",
        )
        builder.ConnectInput("gen3.state", gamepad.GetInputPort("robot_state"))
        builder.ExportOutput(diff_ik.GetOutputPort("joint_positions"), "gen3.position")
        builder.ExportOutput(
            gamepad.GetOutputPort("gripper_command"), "gripper.position"
        )
        builder.BuildInto(self)


class GamepadPoseIntegrator(LeafSystem):
    def __init__(self, meshcat: Meshcat, controller_plant: MultibodyPlant):
        super().__init__()
        self._meshcat = meshcat
        self._time_step = 0.005
        self.linear_speed = 0.1
        self.angular_speed = 0.8
        self.controller_plant = controller_plant
        self.world_frame = controller_plant.world_frame()
        self.ee_frame = controller_plant.GetFrameByName("end_effector_frame")
        self.plant_context = controller_plant.CreateDefaultContext()

        self.X_WE_desired_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(RigidTransform())
        )
        self.linear_mode_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(True)
        )  # gamepad mode

        self.gripper_pos_state_idx = self.DeclareDiscreteState(1)

        self.robot_state_port = self.DeclareVectorInputPort(
            "robot_state", controller_plant.num_multibody_states()
        )
        self.DeclareStateOutputPort("X_WE_desired", self.X_WE_desired_state_idx)
        self.DeclareStateOutputPort("gripper_command", self.gripper_pos_state_idx)
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
                    np.array([-left[0], left[1],right[1],]) * self.angular_speed
                )

        X_WE_desired.set_translation(
            X_WE_desired.translation() + target_twist[3:] * self._time_step
        )

        R_delta = RotationMatrix(RollPitchYaw(target_twist[:3] * self._time_step))
        X_WE_desired.set_rotation(
            X_WE_desired.rotation().multiply(R_delta).ToQuaternion()
        )

        cmd_pos = np.copy(discrete_state.get_value(self.gripper_pos_state_idx))
        if not gamepad.index == None:
            gripper_close = gamepad.button_values[6] * 3
            gripper_open = gamepad.button_values[7] * 3
            pos_delta = np.array([(gripper_close - gripper_open) / 2])*self._time_step
            cmd_pos = np.clip(cmd_pos+pos_delta, 0, 1)

        context.SetAbstractState(self.X_WE_desired_state_idx, X_WE_desired)
        discrete_state.set_value(self.gripper_pos_state_idx, cmd_pos)


        viz_color = Rgba(0,0.5,0,0.5) if linear_mode else Rgba(0,0,0.5,0.5)
        self._meshcat.SetObject("ee", Sphere(0.05), viz_color)
        # self._meshcat.SetObject("target", Mesh("/home/ksuresh/piplup/models/robotiq_description/meshes/visual/robotiq_arg2f_85_base_link.obj",1), viz_color)

        X_WE = copy(X_WE_desired)
        self._meshcat.SetTransform("/drake/ee", X_WE)
        # X_WE.set_rotation(
        #     X_WE_desired.rotation().multiply(
        #         RotationMatrix(RollPitchYaw([0, 0, np.pi / 2]))
        #     )
        # )
        self._meshcat.SetTransform("/drake/target", X_WE)
