from pydrake.all import *
from enum import Enum, auto
from epick_interface import ObjectDetectionStatus

from kinova_gen3 import Gen3ControlMode, Gen3NamedPosition


class PorosityPlannerState(Enum):
    INIT = auto()
    MOVE_TO_NEUTRAL = auto()
    SCAN_MANIPULAND = auto()
    CALC_PICK_POSE = auto()
    MOVE_TO_PRE_PICK = auto()
    MOVE_TO_PICK = auto()


class PorosityPlanner(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # States
        self.planner_state_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(PorosityPlannerState.INIT)
        )
        self.control_mode_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(Gen3ControlMode.kTwist)
        )
        self.command_idx_ = self.DeclareDiscreteState(7)

        # Inputs Ports
        # self.pressure_idx_ = self.DeclareVectorInputPort(
        #     "actual_vacuum_pressure", 1
        # ).get_index()
        # self.obj_det_idx_ = self.DeclareAbstractInputPort(
        #     "object_detection_status", AbstractValue.Make(ObjectDetectionStatus(0))
        # ).get_index()

        # Output Ports
        self.DeclareVectorOutputPort("arm_command", 7, self.CalcGen3Command)
        self.DeclareStateOutputPort("control_mode", self.control_mode_idx_)

        # self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)

    def change_planner_state(self, state: State, new_state: PorosityPlannerState):
        state.get_mutable_abstract_state(self.planner_state_idx_).set_value(new_state)

    def Update(self, context: Context, state: State):
        planner_state = context.get_abstract_state(self.planner_state_idx_).get_value()
        print(planner_state)
        match planner_state:
            case PorosityPlannerState.INIT:
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    np.zeros(7)
                )

                # state.get_mutable_abstract_state(self.planner_state_idx_).set_value(
                #     PorosityPlannerState.MOVE_TO_NEUTRAL
                # )
            case PorosityPlannerState.MOVE_TO_NEUTRAL:
                state.get_mutable_abstract_state(self.control_mode_idx_).set_value(
                    Gen3ControlMode.kPosition
                )
                state.get_mutable_abstract_state(self.planner_state_idx_).set_value(
                    PorosityPlannerState.SCAN_MANIPULAND
                )
            case PorosityPlannerState.SCAN_MANIPULAND:
                # TODO Eval the point cloud port
                state.get_mutable_abstract_state(self.planner_state_idx_).set_value(
                    PorosityPlannerState.CALC_PICK_POSE
                )
            case PorosityPlannerState.CALC_PICK_POSE:
                state.get_mutable_abstract_state(self.planner_state_idx_).set_value(
                    PorosityPlannerState.MOVE_TO_PRE_PICK
                )
            case PorosityPlannerState.MOVE_TO_PRE_PICK:
                state.get_mutable_abstract_state(self.planner_state_idx_).set_value(
                    PorosityPlannerState.GO_TO_HOME
                )
            case PorosityPlannerState.MOVE_TO_PICK:
                state.get_mutable_abstract_state(self.planner_state_idx_).set_value(
                    PorosityPlannerState.GO_TO_HOME
                )
            case _:
                print("Invalid Planner State")

    def CalcGen3Command(self, context: Context, output: BasicVector):
        cmd = context.get_mutable_abstract_state(self.command_idx_).get_value()

        output.SetFromVector(cmd)
