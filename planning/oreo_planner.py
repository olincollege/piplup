from pydrake.all import *
from enum import Enum, auto
from epick_interface import ObjectDetectionStatus  # type: ignore

from kinova_gen3 import Gen3ControlMode, Gen3NamedPosition, kGen3NamedPositions
import time

from common.logging import *

# THINGS TO DO
# [x] Add VR PID back in
# [x] Fix the VR frames
# [] Joint positions
# [] Gripper widths
# [] Timing
# [] Write second arm script


WIDE_FACE_OPEN = 0.4
WIDE_FACE_CLOSED = 0.48
THIN_FACE_OPEN = 0.8
THIN_FACE_CLOSED = 0.95

class OreoPlannerState(Enum):
    INIT = auto()
    MOVE_TO_PRE_PICK = auto()
    WAIT_TO_PICK = auto()
    MOVE_TO_PICK = auto()
    PICK = auto()
    LIFT = auto()
    ROTATE_TO_SEP = auto()
    TWIST = auto()
    MANUAL = auto()
    MOVE_TO_SIDE_PICK = auto()
    SIDE_PICK = auto()
    MOVE_TO_MOUTH = auto()
    MOVE_DOWN = auto()
    IDLE = auto


class OreoPlanner(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # States
        self.planner_state_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(OreoPlannerState.INIT)
        )
        self.control_mode_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(Gen3ControlMode.kTwist)
        )
        self.command_idx_ = self.DeclareDiscreteState(7)
        self.gripper_command_idx_ = self.DeclareDiscreteState(1)

        # Inputs Ports
        self.arm_pose_port_ = self.DeclareVectorInputPort("pose_measured", 6)
        self.vr_command_port_ = self.DeclareVectorInputPort("vr_command", 6)
        self.vr_gripper_command_port_ = self.DeclareVectorInputPort("vr_gripper", 1)
        self.vr_buttons_port_ = self.DeclareAbstractInputPort("vr_buttons", AbstractValue.Make(dict()))

        # Output Ports
        self.DeclareStateOutputPort("arm_command", self.command_idx_)
        self.DeclareStateOutputPort("control_mode", self.control_mode_idx_)
        self.DeclareStateOutputPort("gripper_command", self.gripper_command_idx_)

        # self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.0001, 0.0, self.Update)
        self.prev_state = None

    def change_planner_state(self, state: State, new_state: OreoPlannerState):
        self.prev_state = state.get_abstract_state(self.planner_state_idx_).get_value()
        state.get_mutable_abstract_state(self.planner_state_idx_).set_value(new_state)

    def change_command_mode(self, state: State, new_mode: Gen3ControlMode):
        state.get_mutable_abstract_state(self.control_mode_idx_).set_value(new_mode)

    def Update(self, context: Context, state: State):
        planner_state = context.get_abstract_state(self.planner_state_idx_).get_value()
        logging.info(f"Current State: {planner_state}, Previous State: {self.prev_state}")

        match planner_state:
            case OreoPlannerState.INIT:
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.radians(np.array([7.0, 37.3, 348.7, 71.8, 5.0, 70.33, 37.0]))
                )
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([WIDE_FACE_OPEN])
                )
                self.last_time = context.get_time()
                self.change_planner_state(state, OreoPlannerState.MOVE_TO_PRE_PICK)
            case OreoPlannerState.MOVE_TO_PRE_PICK:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.array([0, 3.14, np.pi/4, 0.55, 0, 0.23, 0])
                )
                if (context.get_time() - self.last_time) > 0.0005:
                    self.change_planner_state(state, OreoPlannerState.WAIT_TO_PICK)
            case OreoPlannerState.WAIT_TO_PICK:
                input("Enter to Start Sequence...")
                self.change_planner_state(state, OreoPlannerState.MOVE_TO_PICK)
            case OreoPlannerState.MOVE_TO_PICK:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.array([0, 3.14, np.pi/4, 0.55, 0, 0.078, 0])
                )
                self.change_planner_state(state, OreoPlannerState.PICK)
                self.last_time = context.get_time()
            case OreoPlannerState.PICK:
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([WIDE_FACE_CLOSED])
                )
                if (context.get_time() - self.last_time) > 0.00005:
                    self.change_planner_state(state, OreoPlannerState.LIFT)
            case OreoPlannerState.LIFT:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.array([0, 3.14, np.pi/4, 0.55, 0, 0.15, 0])
                )
                self.change_planner_state(state, OreoPlannerState.ROTATE_TO_SEP)
            case OreoPlannerState.ROTATE_TO_SEP:
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.radians(np.array([332.0, 70.2, 11.1, 86.8, 299.1, 288.6, 95.2]))
                )
                self.change_planner_state(state, OreoPlannerState.TWIST)
            case OreoPlannerState.TWIST:
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.radians(np.array([331.0, 70.2, 11.1, 86.8, 299.1, 288.6, 52.2]))
                )
                # self.change_planner_state(state, OreoPlannerState.MANUAL)
            case OreoPlannerState.MANUAL:
                self.change_command_mode(state, Gen3ControlMode.kTwist)
                
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.append(self.vr_command_port_.Eval(context), [0.0])
                )
                # state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                #     self.vr_gripper_command_port_.Eval(context)
                # )
                buttons = self.vr_buttons_port_.Eval(context)
                if 'B' in buttons and buttons['B']:
                    if self.prev_state == OreoPlannerState.TWIST:
                        self.change_planner_state(state, OreoPlannerState.MOVE_TO_SIDE_PICK)
                    else:
                        self.change_planner_state(state, OreoPlannerState.MOVE_TO_MOUTH)
            case OreoPlannerState.MOVE_TO_SIDE_PICK:
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.array([0.0, 0.477, 0.0, 1.32, 0.0, 1.33, 0.0])
                )
                self.change_planner_state(state, OreoPlannerState.SIDE_PICK)
            case OreoPlannerState.SIDE_PICK:
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([THIN_FACE_CLOSED])
                )
                self.change_planner_state(state, OreoPlannerState.MANUAL)
            case OreoPlannerState.MOVE_TO_MOUTH:
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.array([0.0, 0.477, 0.0, 1.32, 0.0, 1.33, 0.0])
                )
                self.change_planner_state(state, OreoPlannerState.MOVE_DOWN)
            case OreoPlannerState.MOVE_DOWN:
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.array([0.0, 0.477, 0.0, 1.32, 0.0, 1.33, 0.0])
                )
                self.change_planner_state(state, OreoPlannerState.IDLE)
            case OreoPlannerState.IDLE:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.array([0, 3.14, np.pi/2, 0.55, 0, 0.25, 0])
                )
            case _:
                logging.error("Invalid Planner State")

    def CalcGen3Command(self, context: Context, output: BasicVector):
        cmd = context.get_discrete_state(self.command_idx_)
        output.SetFromVector(cmd)
