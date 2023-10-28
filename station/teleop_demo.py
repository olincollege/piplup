import argparse
from typing import List

import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import *
from pydrake.common import RandomGenerator
from pydrake.geometry import Meshcat, StartMeshcat
from pydrake.manipulation._manipulation_extra import ApplyDriverConfigs
from pydrake.multibody.parsing import ModelDirectives, ProcessModelDirectives
from pydrake.multibody.plant import AddMultibodyPlant
from pydrake.systems.analysis import ApplySimulatorConfig, Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import ApplyCameraConfig
from pydrake.visualization import ApplyVisualizationConfig
from scenario import Scenario, _load_scenario

from common import ConfigureParser
from kinova_gen3 import GamepadDiffIkController
from station import MakeHardwareStation


def run(*, scenario: Scenario, graphviz=None):
    """Runs a simulation of the given scenario."""
    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    hardware_station: Diagram = builder.AddNamedSystem(
        "hardware_station", MakeHardwareStation(scenario, meshcat)
    )

    # TODO (krishna) These should be more generic
    controller_plant: MultibodyPlant = hardware_station.GetSubsystemByName(
        "gen3_controller_plant"
    ).get()

    gamepad: GamepadDiffIkController = builder.AddNamedSystem(
        "gamepad_control", GamepadDiffIkController(meshcat, controller_plant)
    )
    builder.Connect(
        gamepad.GetOutputPort("gen3.position"),
        hardware_station.GetInputPort("gen3.position"),
    )
    builder.Connect(
        gamepad.GetOutputPort("gripper.position"),
        hardware_station.GetInputPort("2f_85.position"),
    )

    builder.Connect(
        hardware_station.GetOutputPort("gen3.state_estimated"),
        gamepad.GetInputPort("gen3.state"),
    )

    # Build the diagram and its simulator.
    diagram: Diagram = builder.Build()

    meshcat.SetObject("ee", Sphere(0.05), Rgba(0, 0.5, 0, 0.5))
    ee_base = Mesh(
        "models/robotiq_description/meshes/visual/robotiq_arg2f_85_base_link.obj", 1
    )
    meshcat.SetObject("target", ee_base, Rgba(0, 0.5, 0, 0.5))
    meshcat.SetCameraPose(np.array([1, -1, 1]) * 0.75, np.array([0, 0, 0.4]))
    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    # Sample the random elements of the context.
    random = RandomGenerator(scenario.random_seed)
    diagram.SetRandomContext(simulator.get_mutable_context(), random)

    # Visualize the diagram, when requested.
    options = {"plant/split": "I/O"}
    if graphviz is not None:
        with open(graphviz, "w", encoding="utf-8") as f:
            f.write(diagram.GetGraphvizString(options=options))

    plt.figure()
    plot_system_graphviz(diagram, max_depth=1, options=options)
    plt.show()

    # Simulate.
    simulator.AdvanceTo(scenario.simulation_duration)


def main():
    scenario = _load_scenario(
        filename="models/example_scenarios.yaml",
        scenario_name="Demo",
    )
    run(scenario=scenario, graphviz=None)


if __name__ == "__main__":
    main()
