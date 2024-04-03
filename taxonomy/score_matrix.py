from pydrake.all import *
from enum import Enum, auto

from kinova_gen3 import Gen3ControlMode, Gen3NamedPosition, kGen3NamedPositions
import time

from common.logging import *

class ScoreMatrixEvaluator(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # States
        self.evaluator_state_idx = self.DeclareAbstractState(
            AbstractValue.Make(ScoreMatrixEvaluator.INIT)
        )
        self.control_mode_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(Gen3ControlMode.kTwist)
        )
        self.command_idx_ = self.DeclareDiscreteState(7)
        self.gripper_command_idx_ = self.DeclareDiscreteState(1)

        # Input Ports


        # Output Ports
        self.DeclareStateOutputPort("arm_command", self.command_idx_)
        self.DeclareStateOutputPort("control_mode", self.control_mode_idx_)
        self.DeclareStateOutputPort("gripper_command", self.gripper_command_idx_)

        self.DeclarePeriodicUnrestrictedUpdateEvent(0.0001, 0.0, self.Update)
        
    def update(self):
        pass