from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
import numpy as np
from common import ConfigureParser
from suction_gripper import CupPressureSource, CupObjInterface
from .epick_control import AddSimEPickDriver

# Driver
class EPickDriver:
    __fields__: ClassVar[tuple] = (
        SimpleNamespace(name="cup_frames", type=List[str]),
    )
    cup_frames: List[str]

def ApplyDriverConfig(
    driver_config: EPickDriver,
    model_instance_name: str,
    sim_plant: MultibodyPlant,
    models_from_directives_map: List[ModelInstanceInfo],
    lcm_buses: LcmBuses,
    builder: DiagramBuilder,
):
    suction_gripper = models_from_directives_map["suction_gripper"]
    obj_model_info :ModelInstanceInfo = models_from_directives_map["cracker_box"]
    obj_body = sim_plant.GetBodyByName("base_link_cracker", obj_model_info.model_instance)

    diag = AddSimEPickDriver(sim_plant, suction_gripper,obj_body ,builder)

    builder.Connect(diag.GetOutputPort("suction_force"), sim_plant.get_applied_spatial_force_input_port())