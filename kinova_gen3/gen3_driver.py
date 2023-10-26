from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from .gen3_constants import *
import numpy as np
from .gen3_control import BuildGen3Control, AddSimGen3Driver


# Driver
class Gen3Driver:
    __fields__: ClassVar[tuple] = (
        SimpleNamespace(name="hand_model_name", type=str),
        SimpleNamespace(name="control_level", type=Gen3ControlLevel),
        SimpleNamespace(name="control_mode", type=Gen3JointControlMode),
        SimpleNamespace(name="ip_address", type=str),
        SimpleNamespace(name="port", type=int),
    )
    hand_model_name: str
    control_level: Gen3ControlLevel = Gen3ControlLevel.kHighLevel
    control_mode: Gen3JointControlMode = Gen3JointControlMode.kPosition
    ip_address: str = None
    port: int = None


# Driver Functions
def ApplyDriverConfig(
    driver_config: Gen3Driver,
    model_instance_name: str,
    sim_plant: MultibodyPlant,
    models_from_directives_map: List[ModelInstanceInfo],
    lcm_buses: LcmBuses,
    builder: DiagramBuilder,
):
    gen3_model: ModelInstanceInfo = models_from_directives_map[model_instance_name]
    controller_plant = MultibodyPlant(0.0)
    parser = Parser(controller_plant)
    controller_models: List[ModelInstanceIndex] = parser.AddModels(
        gen3_model.model_path
    )
    assert len(controller_models) == 1
    gen3_controller_model_idx = controller_models[0]

    controller_plant.WeldFrames(
        controller_plant.world_frame(),
        controller_plant.GetFrameByName(
            gen3_model.child_frame_name, gen3_controller_model_idx
        ),
        RigidTransform(),
    )

    controller_plant.Finalize()

    builder.AddNamedSystem(
        f"{model_instance_name}_controller_plant", SharedPointerSystem(controller_plant)
    )
    if driver_config.ip_address and driver_config.port:
        pass  # TODO BuildGen3Control
    else:
        AddSimGen3Driver(
            sim_plant, gen3_model.model_instance, controller_plant, builder
        )
