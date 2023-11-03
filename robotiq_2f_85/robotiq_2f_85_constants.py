from enum import Enum, auto
import numpy as np


class GripperTarget(Enum):
    kPosition = auto()
    kVelocity = auto()
