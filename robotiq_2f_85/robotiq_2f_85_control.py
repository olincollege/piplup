from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from .robotiq_2f_85_constants import *
from .robotiq_2f_85_sim_driver import Sim2f85Driver
import numpy as np


def AddSim2f85Driver(
    plant: MultibodyPlant,
    gripper_instance: ModelInstanceIndex,
    controller_plant: MultibodyPlant,
    builder: DiagramBuilder,
) -> System:
    inner_name = f"2f85Driver({plant.GetModelInstanceName(gripper_instance)})"
    system: Sim2f85Driver = builder.AddNamedSystem(
        inner_name, Sim2f85Driver(controller_plant)
    )
    builder.Connect(
        plant.get_state_output_port(gripper_instance), system.GetInputPort("state")
    )
    builder.Connect(
        system.GetOutputPort("applied_gripper_torque"),
        plant.get_actuation_input_port(gripper_instance),
    )
    return system
