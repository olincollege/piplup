from enum import Enum, auto
import numpy as np
from pydrake.multibody.plant import MultibodyPlant


# Constants
class Gen3ControlLevel(Enum):
    kHighLevel = auto()  # Upto 40Hz
    kLowLevel = auto()  # Upto 1kHz
    kLowLevelBypass = auto()  # Upto 1kHz


class Gen3ControlMode(Enum):
    kPosition = auto()
    kVelocity = auto()
    # kTorque = auto()
    kPose = auto()
    kTwist = auto()
    # kWrench = auto()


class Gen3NamedPosition(Enum):
    SLEEP = auto()
    NEUTRAL = auto()
    CAMCLEAR = auto()


kGen3NamedPositions: dict[Gen3NamedPosition, np.ndarray] = {
    Gen3NamedPosition.NEUTRAL: np.array([0.0, 0.477, 0.0, 1.32, 0.0, 1.33, 0.0]),
    Gen3NamedPosition.CAMCLEAR: np.array(
        [0.0, 5.91667, 0.0, 2.30383, 0.0, 1.13446, 0.0]
    ),
    Gen3NamedPosition.SLEEP: np.array([]),
}

kGen3ArmNumJoints: int = 7
kGen3KortexAPIPeriod: float = 0.025  # 40hz


def get_gen3_joint_velocity_limits(plant: MultibodyPlant):
    qd_min = []
    qd_max = []

    joint_indices = plant.GetJointIndices(plant.GetModelInstanceByName("Kinova_Gen3"))

    for idx in joint_indices:
        joint = plant.get_joint(idx)

        if joint.type_name() == "revolute":  # ignore the joint welded to the world
            qd_min.append(joint.velocity_lower_limit())
            qd_max.append(joint.velocity_upper_limit())

    return (np.array(qd_min), np.array(qd_max))


def get_gen3_joint_position_limits(plant: MultibodyPlant):
    q_min = []
    q_max = []

    joint_indices = plant.GetJointIndices(plant.GetModelInstanceByName("Kinova_Gen3"))

    for idx in joint_indices:
        joint = plant.get_joint(idx)

        if joint.type_name() == "revolute":  # ignore the joint welded to the world
            q_min.append(joint.position_lower_limit())
            q_max.append(joint.position_upper_limit())

    return (np.array(q_min), np.array(q_max))
