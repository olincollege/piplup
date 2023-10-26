from enum import Enum, auto

# Constants
class Gen3ControlLevel(Enum):
    kHighLevel = auto()  # Upto 40Hz
    kLowLevel = auto()  # Upto 1kHz
    kLowLevelBypass = auto()  # Upto 1kHz


class Gen3JointControlMode(Enum):
    kPosition = auto()
    kVelocity = auto()
    kTorque = auto()

kGen3ArmNumJoints : int = 7
kGen3KortexAPIPeriod : float = 0.025 # 40hz