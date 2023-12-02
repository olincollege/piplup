from typing import Any, ClassVar, List, Optional
import sys
from types import SimpleNamespace
from pydrake.all import *
from .gen3_constants import *
import numpy as np
from .gen3_sim_driver import SimGen3Driver


def AddSimGen3Driver(
    plant: MultibodyPlant,
    gen3_instance: ModelInstanceIndex,
    controller_plant: MultibodyPlant,
    builder: DiagramBuilder,
    control_mode : Gen3ControlMode,
) -> System:
    inner_name = f"Gen3Driver({plant.GetModelInstanceName(gen3_instance)})"
    system: SimGen3Driver = builder.AddNamedSystem(
        inner_name, SimGen3Driver(controller_plant, control_mode)
    )
    builder.Connect(
        plant.get_state_output_port(gen3_instance), system.GetInputPort("state")
    )
    builder.Connect(
        plant.get_generalized_contact_forces_output_port(gen3_instance),
        system.GetInputPort("generalized_contact_forces"),
    )
    builder.Connect(
        system.GetOutputPort("actuation"), plant.get_actuation_input_port(gen3_instance)
    )

    return system
