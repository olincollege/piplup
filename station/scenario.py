import dataclasses as dc
import math
import typing

from pydrake.all import *
from pydrake.common.yaml import yaml_load_typed
from pydrake.lcm import DrakeLcmParams


from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.multibody.parsing import ModelDirective

from pydrake.systems.analysis import SimulatorConfig
from pydrake.systems.sensors import CameraConfig
from pydrake.visualization import VisualizationConfig


from kinova_gen3 import Gen3Driver, Gen3InterfaceConfig
from robotiq_epick import EPickDriver
from realsensed400 import RealsenseInterfaceConfig  # type: ignore


@dc.dataclass
class Scenario:
    random_seed: int = 0

    simulation_duration: float = math.inf

    simulator_config: SimulatorConfig = SimulatorConfig(
        max_step_size=1e-3, accuracy=1.0e-2, target_realtime_rate=1.0
    )

    plant_config: MultibodyPlantConfig = MultibodyPlantConfig()

    directives: typing.List[ModelDirective] = dc.field(default_factory=list)

    model_drivers: typing.Mapping[
        str,
        typing.Union[
            Gen3Driver,
            EPickDriver,
            ZeroForceDriver,
        ],
    ] = dc.field(default_factory=dict)

    cameras: typing.Mapping[str, CameraConfig] = dc.field(default_factory=dict)

    visualization: VisualizationConfig = VisualizationConfig()

    hardware_interface: typing.Mapping[
        str,
        typing.Union[
            Gen3InterfaceConfig,
            RealsenseInterfaceConfig,
        ],
    ] = dc.field(default_factory=dict)


def load_scenario(*, filename, scenario_name):
    """Implements the command-line handling logic for scenario data.
    Returns a `Scenario` object loaded from the given input arguments.
    """
    result = yaml_load_typed(
        schema=Scenario,
        filename=filename,
        child_name=scenario_name,
        defaults=Scenario(),
    )
    return result
