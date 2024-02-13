from pydrake.all import *
from enum import Enum, auto


class PorosityPlannerState(Enum):
    GO_TO_HOME = auto()
    PRE_PICK = auto()
    TEST_SUCTION = auto()


class PorosityPlanner(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        self.mode_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(PorosityPlannerState.GO_TO_HOME)
        )

        # self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)

    def Update(self, context: Context, state: State):
        mode = context.get_abstract_state(self.mode_idx_).get_value()
        print(mode)
        match mode:
            case PorosityPlannerState.GO_TO_HOME:
                state.get_mutable_abstract_state(self.mode_idx_).set_value(
                    PorosityPlannerState.PRE_PICK
                )
            case PorosityPlannerState.PRE_PICK:
                state.get_mutable_abstract_state(self.mode_idx_).set_value(
                    PorosityPlannerState.TEST_SUCTION
                )
            case PorosityPlannerState.TEST_SUCTION:
                state.get_mutable_abstract_state(self.mode_idx_).set_value(
                    PorosityPlannerState.GO_TO_HOME
                )
            case _:
                print("Invalid Planner State")
