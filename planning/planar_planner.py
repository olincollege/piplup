from pydrake.all import *
from enum import Enum, auto

from kinova_gen3 import Gen3ControlMode, Gen3NamedPosition, kGen3NamedPositions
import time

from common.logging import *


class PlanarPlannerState(Enum):
    INIT = auto()
    MOVE_TO_NEUTRAL = auto()
    MOVE_TO_PRE_SCAN = auto()
    SCAN_MANIPULAND = auto()
    CALC_PICK_POSE = auto()
    MOVE_TO_PRE_PICK = auto()
    MOVE_TO_PICK = auto()
    CLOSE_GRIPPER = auto()
    PICK_MANIPULAND = auto()
    DROP_MANIPULAND = auto()
    IDLE = auto()


class PlanarPlanner(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # States
        self.planner_state_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(PlanarPlannerState.INIT)
        )
        self.control_mode_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(Gen3ControlMode.kTwist)
        )
        self.command_idx_ = self.DeclareDiscreteState(7)
        self.gripper_command_idx_ = self.DeclareDiscreteState(1)

        # Input Ports
        self.planar_grasp_port_ = self.DeclareAbstractInputPort(
            "planar_grasp_selection",
            AbstractValue.Make((np.inf, np.inf, RigidTransform(), np.inf)),
        )

        # Output Ports
        self.DeclareStateOutputPort("arm_command", self.command_idx_)
        self.DeclareStateOutputPort("control_mode", self.control_mode_idx_)
        self.DeclareStateOutputPort("gripper_command", self.gripper_command_idx_)

        self.DeclarePeriodicUnrestrictedUpdateEvent(0.0001, 0.0, self.Update)

    def change_planner_state(self, state: State, new_state: PlanarPlannerState):
        state.get_mutable_abstract_state(self.planner_state_idx_).set_value(new_state)

    def change_command_mode(self, state: State, new_mode: Gen3ControlMode):
        state.get_mutable_abstract_state(self.control_mode_idx_).set_value(new_mode)

    def Update(self, context: Context, state: State):
        planner_state = context.get_abstract_state(self.planner_state_idx_).get_value()
        logging.info(f"Current State: {planner_state}")

        match planner_state:
            case PlanarPlannerState.INIT:
                self.change_planner_state(state, PlanarPlannerState.MOVE_TO_NEUTRAL)
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([0.0])
                )
            case PlanarPlannerState.MOVE_TO_NEUTRAL:
                time.sleep(2)
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    kGen3NamedPositions[Gen3NamedPosition.NEUTRAL]
                )
                self.change_planner_state(state, PlanarPlannerState.MOVE_TO_PRE_SCAN)
            case PlanarPlannerState.MOVE_TO_PRE_SCAN:
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    kGen3NamedPositions[Gen3NamedPosition.CAMCLEAR]
                )
                self.change_planner_state(state, PlanarPlannerState.SCAN_MANIPULAND)
            case PlanarPlannerState.SCAN_MANIPULAND:
                self.change_planner_state(state, PlanarPlannerState.CALC_PICK_POSE)
            case PlanarPlannerState.CALC_PICK_POSE:
                self.planar_grasp_selection = self.planar_grasp_port_.Eval(context)
                self.pick_pose_transform: RigidTransform = self.planar_grasp_selection[2] @ RigidTransform([0.0, 0.0, 0.1325])
                self.grip_width = self.planar_grasp_selection[3]
                if np.isinf(self.planar_grasp_selection[0]):
                    self.change_planner_state(state, PlanarPlannerState.IDLE)
                else:
                    self.change_planner_state(state, PlanarPlannerState.MOVE_TO_PRE_PICK)
            case PlanarPlannerState.MOVE_TO_PRE_PICK:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                rotation_matrix: RotationMatrix = self.pick_pose_transform.rotation() @ RollPitchYaw([0, 0, -np.pi/2]).ToRotationMatrix()
                translation: np.ndarray = self.pick_pose_transform.translation()
                self.pick_pose_array = np.concatenate(
                    [rotation_matrix.ToRollPitchYaw().vector(), translation, [0]]
                )
                logging.info(f"Pick pose: {self.pick_pose_array}")
                # Move along -z axis of gripper by 5cm
                pre_pick_offset = rotation_matrix.col(2) * -0.05
                pre_pick_pose = copy.copy(self.pick_pose_array)
                pre_pick_pose[3:6] += pre_pick_offset

                logging.info(f"Pre-pick pose: {pre_pick_pose}")

                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    pre_pick_pose
                )

                self.change_planner_state(state, PlanarPlannerState.MOVE_TO_PICK)
            case PlanarPlannerState.MOVE_TO_PICK:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                logging.info(f"Pick pose: {self.pick_pose_array}")
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    self.pick_pose_array
                )
                self.change_planner_state(state, PlanarPlannerState.CLOSE_GRIPPER)
            case PlanarPlannerState.CLOSE_GRIPPER:
                gripper_command = np.clip(1 - self.grip_width / 0.085 + 0.2, 0, 1)
                logging.info(f"Gripper command: {gripper_command}")
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([gripper_command])
                )
                self.change_planner_state(state, PlanarPlannerState.PICK_MANIPULAND)
                self.last_time = context.get_time()
            case PlanarPlannerState.PICK_MANIPULAND:
                if (context.get_time() - self.last_time) > 0.0005:
                    self.change_command_mode(state, Gen3ControlMode.kPose)
                    pick_up_pose = copy.copy(self.pick_pose_array)
                    pick_up_pose[5] += 0.1
                    state.get_mutable_discrete_state(self.command_idx_).set_value(
                        pick_up_pose
                    )
                    self.change_planner_state(state, PlanarPlannerState.DROP_MANIPULAND)
            case PlanarPlannerState.DROP_MANIPULAND:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                logging.info(f"Pick pose: {self.pick_pose_array}")
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    self.pick_pose_array
                )
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([0.0])
                )
                self.change_planner_state(state, PlanarPlannerState.IDLE)
                self.last_time = context.get_time()
            case PlanarPlannerState.IDLE:
                if (context.get_time() - self.last_time) > 0.001:
                    self.change_command_mode(state, Gen3ControlMode.kPosition)
                    state.get_mutable_discrete_state(self.command_idx_).set_value(
                        kGen3NamedPositions[Gen3NamedPosition.NEUTRAL]
                    )
            case _:
                logging.error("Invalid Planner State")

    # def CalcGen3Command(self, context: Context, output: BasicVector):
    #     cmd = context.get_discrete_state(self.command_idx_)
    #     output.SetFromVector(cmd)
