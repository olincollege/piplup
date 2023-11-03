from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
import numpy as np
from common import ConfigureParser
from .epick_control import AddSimEPickDriver


# Driver
class EPickDriver:
    __fields__: ClassVar[tuple] = (
        SimpleNamespace(name="num_cups", type=int),
        SimpleNamespace(name="interaction_bodies", type=List[str]),
    )
    num_cups: int
    interaction_bodies: List[str]


def ApplyDriverConfig(
    driver_config: EPickDriver,
    model_instance_name: str,
    sim_plant: MultibodyPlant,
    models_from_directives_map: List[ModelInstanceInfo],
    lcm_buses: LcmBuses,
    builder: DiagramBuilder,
):
    # TODO if hardware apply epick modbus interface (krishna)

    gripper_model: ModelInstanceInfo = models_from_directives_map[model_instance_name]
    obj_bodies = []
    for body_name in driver_config.interaction_bodies:
        body = sim_plant.GetBodyByName(body_name)
        if body == None:
            raise RuntimeError(
                f"Interaction body for suction gripper not found: {body_name}"
            )
        obj_bodies.append(body)

    if not obj_bodies:
        raise RuntimeError(f"At least 1 interaction body must be added")

    diag = AddSimEPickDriver(
        sim_plant,
        gripper_model.model_instance,
        obj_bodies,
        builder,
    )

    builder.Connect(
        diag.GetOutputPort("suction_force"),
        sim_plant.get_applied_spatial_force_input_port(),
    )
