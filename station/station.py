import argparse
import dataclasses as dc
import math
import typing
import os
import matplotlib.pyplot as plt
import numpy as np

from pydrake.all import *
from pydrake.common import RandomGenerator
from pydrake.common.yaml import yaml_load_typed
from pydrake.lcm import DrakeLcmParams

from pydrake.manipulation._manipulation_extra import ApplyDriverConfigs

from pydrake.multibody.plant import (
    AddMultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.multibody.parsing import (
    ModelDirective,
    ModelDirectives,
    ProcessModelDirectives,
)
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import ApplyLcmBusConfig
from pydrake.systems.sensors import (
    ApplyCameraConfig,
    CameraConfig,
)
from pydrake.visualization import (
    ApplyVisualizationConfig,
    VisualizationConfig,
)

from pydrake.geometry import StartMeshcat

from kinova_gen3 import Gen3Driver, GamepadDiffIkController


def ConfigureParser(parser: Parser):
    """Add the manipulation/package.xml index to the given Parser."""
    package_xml = "/home/ksuresh/piplup/models/package.xml"
    parser.package_map().AddPackageXml(filename=package_xml)


@dc.dataclass
class Scenario:
    """Defines the YAML format for a (possibly stochastic) scenario to be
    simulated.
    """

    # Random seed for any random elements in the scenario. The seed is always
    # deterministic in the `Scenario`; a caller who wants randomness must
    # populate this value from their own randomness.
    random_seed: int = 0

    # The maximum simulation time (in seconds).  The simulator will attempt to
    # run until this time and then terminate.
    simulation_duration: float = math.inf

    # Simulator configuration (integrator and publisher parameters).
    simulator_config: SimulatorConfig = SimulatorConfig(
        max_step_size=1e-3, accuracy=1.0e-2, target_realtime_rate=1.0
    )

    # Plant configuration (time step and contact parameters).
    plant_config: MultibodyPlantConfig = MultibodyPlantConfig()

    # All of the fully deterministic elements of the simulation.
    directives: typing.List[ModelDirective] = dc.field(default_factory=list)

    # A map of {bus_name: lcm_params} for LCM transceivers to be used by
    # drivers, sensors, etc.
    lcm_buses: typing.Mapping[str, DrakeLcmParams] = dc.field(
        default_factory=lambda: dict(default=DrakeLcmParams())
    )

    # For actuated models, specifies where each model's actuation inputs come
    # from, keyed on the ModelInstance name.
    model_drivers: typing.Mapping[
        str,
        typing.Union[
            Gen3Driver,
            ZeroForceDriver,
        ],
    ] = dc.field(default_factory=dict)

    # Cameras to add to the scene (and broadcast over LCM). The key for each
    # camera is a helpful mnemonic, but does not serve a technical role. The
    # CameraConfig::name field is still the name that will appear in the
    # Diagram artifacts.
    cameras: typing.Mapping[str, CameraConfig] = dc.field(default_factory=dict)

    visualization: VisualizationConfig = VisualizationConfig()


def _load_scenario(*, filename, scenario_name, scenario_text):
    """Implements the command-line handling logic for scenario data.
    Returns a `Scenario` object loaded from the given input arguments.
    """
    result = yaml_load_typed(
        schema=Scenario,
        filename=filename,
        child_name=scenario_name,
        defaults=Scenario(),
    )
    result = yaml_load_typed(schema=Scenario, data=scenario_text, defaults=result)
    return result


def run(*, scenario: Scenario, graphviz=None):
    """Runs a simulation of the given scenario."""
    builder = DiagramBuilder()

    # Create the multibody plant and scene graph.
    sim_plant, scene_graph = AddMultibodyPlant(
        config=scenario.plant_config, builder=builder
    )
    parser = Parser(sim_plant)
    ConfigureParser(parser)
    # Add model directives.
    added_models = ProcessModelDirectives(
        directives=ModelDirectives(directives=scenario.directives), parser=parser
    )

    # Now the plant is complete.
    sim_plant.Finalize()

    lcm_buses = None

    # Add actuation inputs.
    ApplyDriverConfigs(
        driver_configs=scenario.model_drivers,
        sim_plant=sim_plant,
        models_from_directives=added_models,
        lcm_buses=lcm_buses,
        builder=builder,
    )

    # Add scene cameras.
    for _, camera in scenario.cameras.items():
        ApplyCameraConfig(config=camera, builder=builder, lcm_buses=lcm_buses)

    # Add visualization.
    meshcat = StartMeshcat()
    ApplyVisualizationConfig(
        scenario.visualization, builder, lcm_buses, meshcat=meshcat
    )
    controller_plant : MultibodyPlant= builder.GetSubsystemByName("gen3_controller_plant").get()
    gen3_driver: System = builder.GetSubsystemByName("Gen3Driver(gen3)")
    gripper_driver: System = builder.GetSubsystemByName("2f85Driver(2f_85)")
    gamepad: GamepadDiffIkController = builder.AddNamedSystem("gamepad_control",
        GamepadDiffIkController(meshcat, controller_plant)
    )
    builder.Connect(
        gamepad.GetOutputPort("gen3.position"), gen3_driver.GetInputPort("position")
    )
    builder.Connect(
        gamepad.GetOutputPort("gripper.velocity"), gripper_driver.GetInputPort("velocity")
    )

    builder.Connect(gen3_driver.GetOutputPort("state_estimated"), gamepad.GetInputPort("gen3.state"))

    # Build the diagram and its simulator.
    diagram: Diagram = builder.Build()

    meshcat.SetObject("test", Sphere(0.05), Rgba(0,0.5,0,0.5))
    ee_base = Mesh("/home/ksuresh/piplup/models/robotiq_description/meshes/visual/robotiq_arg2f_85_base_link.obj",1)
    meshcat.SetObject("target", ee_base, Rgba(0,0.5,0,0.5))
    meshcat.SetCameraPose(np.array([1,-1,1])*0.75, np.array([0,0,0.4]))
    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    # Sample the random elements of the context.
    random = RandomGenerator(scenario.random_seed)
    diagram.SetRandomContext(simulator.get_mutable_context(), random)

    # Visualize the diagram, when requested.
    if graphviz is not None:
        with open(graphviz, "w", encoding="utf-8") as f:
            options = {"plant/split": "I/O"}
            f.write(diagram.GetGraphvizString(options=options))

        plt.figure()
        plot_graphviz(diagram.GetGraphvizString(options=options))
        plt.show()
    # Simulate.
    simulator.AdvanceTo(scenario.simulation_duration)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario_file",
        required=True,
        help="Scenario filename, e.g., "
        "drake/examples/hardware_sim/example_scenarios.yaml",
    )
    parser.add_argument(
        "--scenario_name",
        required=True,
        help="Scenario name within the scenario_file, e.g., Demo in the "
        "example_scenarios.yaml; scenario names appears as the keys of "
        "the YAML document's top-level mapping item",
    )
    parser.add_argument(
        "--scenario_text",
        default="{}",
        help="Additional YAML scenario text to load, in order to override "
        "values in the scenario_file, e.g., timeouts",
    )
    parser.add_argument(
        "--graphviz",
        metavar="FILENAME",
        help="Dump the Simulator's Diagram to this file in Graphviz format "
        "as a debugging aid",
    )
    args = parser.parse_args()
    scenario = _load_scenario(
        filename=args.scenario_file,
        scenario_name=args.scenario_name,
        scenario_text=args.scenario_text,
    )
    run(scenario=scenario, graphviz=args.graphviz)


if __name__ == "__main__":
    main()
