from pydrake.all import *
import numpy as np
from pydrake.geometry import Meshcat, SceneGraph
from .gen3_constants import (
    get_gen3_joint_velocity_limits,
    get_gen3_joint_position_limits,
)


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
            "gamepad", GamepadPoseIntegrator(meshcat)
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
        builder.ExportOutput(diff_ik.GetOutputPort("joint_positions"), "gen3.position")
        builder.ExportOutput(
            gamepad.GetOutputPort("gripper_command"), "gripper.velocity"
        )
        builder.BuildInto(self)


class GamepadPoseIntegrator(LeafSystem):
    def __init__(self, meshcat: Meshcat):
        super().__init__()
        self._meshcat = meshcat
        self._time_step = 0.005
        self.linear_speed = 0.1
        self.angular_speed = 0.8

        self.X_WE_desired_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(RigidTransform())
        )
        self.linear_mode_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(True)
        )  # gamepad mode
        self.DeclareStateOutputPort("X_WE_desired", self.X_WE_desired_state_idx)
        self.DeclareVectorOutputPort(
            "gripper_command", BasicVector(1), self.CalcGripperCommand
        )
        self.DeclarePeriodicDiscreteUpdateEvent(self._time_step, 0, self.Integrate)
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)

    def Initialize(self, context: Context, discrete_state: DiscreteValues):
        # q = self.joint_position_port.Eval(context)
        # self._plant.SetPositions(self._plant_context, q)
        # ee_pose : RigidTransform = self._plant.CalcRelativeTransform(self._plant_context, self.world_frame, self.ee_frame)
        context.SetAbstractState(self.X_WE_desired_state_idx, RigidTransform())

    def Integrate(self, context: Context, discrete_state: DiscreteValues):
        
        X_WE_desired: RigidTransform = context.get_abstract_state(self.X_WE_desired_state_idx).get_value()
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

            # if gamepad.button_values[8]:
            #     self.linear_mode = True
            # if gamepad.button_values[9]:
            #     self.linear_mode = False

            if context.get_abstract_state(self.linear_mode_state_idx).get_value():
                target_twist[3:] = (
                    np.array([left[0], left[1], right[1]]) * self.linear_speed
                )
            else:
                ee_rot = X_WE_desired.rotation()
                target_twist[:3] = ee_rot.multiply(
                    np.array([-left[0], right[1], left[1]]) * self.angular_speed
                )

        X_WE_desired.set_translation(
            X_WE_desired.translation() + target_twist[3:] * self._time_step
        )

        # R_delta = RotationMatrix(RollPitchYaw(target_twist[:3] * self._time_step))
        # X_WE_desired.set_rotation(
        #     X_WE_desired.rotation().multiply(R_delta).ToQuaternion()
        # )

        context.SetAbstractState(self.X_WE_desired_state_idx, X_WE_desired)

        self._meshcat.SetTransform("/drake/test", X_WE_desired)
        # X_WE_desired.set_rotation(X_WE_desired.rotation().multiply(RotationMatrix(RollPitchYaw([0, 0, np.pi / 2]))))
        self._meshcat.SetTransform("/drake/target", X_WE_desired)

    def CalcGripperCommand(self, context, output):
        gamepad = self._meshcat.GetGamepad()

        if gamepad.index == None:
            output.SetFromVector(np.zeros(1))
            return
        gripper_close = gamepad.button_values[6] * 3
        gripper_open = gamepad.button_values[7] * 3
        cmd_pos = np.array([(gripper_close - gripper_open) / 2])
        output.SetFromVector(cmd_pos)
