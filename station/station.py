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
from station.scenario import Scenario

from common import ConfigureParser
from kinova_gen3 import GamepadDiffIkController


def MakeHardwareStation(
    scenario: Scenario,
    meshcat: Meshcat = None,
) -> Diagram:
    if scenario.hardware_interface:
        return MakeHardwareStationInterface(scenario, meshcat)

    builder = DiagramBuilder()

    # Create the multibody plant and scene graph.
    sim_plant: MultibodyPlant
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
    for model_name, driver_config in scenario.model_drivers.items():
        # Find the subsystem with the driver for this model instance.
        driver_type = type(driver_config).__name__
        driver_subsystem: System = builder.GetSubsystemByName(
            f"{driver_type}({model_name})"
        )
        # Export any unconnected input ports.
        for i in range(driver_subsystem.num_input_ports()):
            port = driver_subsystem.get_input_port(i)
            if not builder.IsConnectedOrExported(port):
                if port.get_name() == "geometry_query":
                    builder.Connect(scene_graph.get_query_output_port(), port)
                else:
                    builder.ExportInput(port, f"{model_name}.{port.get_name()}")
        # Export any unconnected output ports.
        for i in range(driver_subsystem.num_output_ports()):
            port = driver_subsystem.get_output_port(i)
            builder.ExportOutput(port, f"{model_name}.{port.get_name()}")

    # Add scene cameras.
    for camera_name, camera in scenario.cameras.items():
        ApplyCameraConfig(config=camera, builder=builder, lcm_buses=lcm_buses)
        camera_subsystem: System = builder.GetSubsystemByName(
            f"rgbd_sensor_{camera_name}"
        )
        for i in range(camera_subsystem.num_output_ports()):
            port = camera_subsystem.get_output_port(i)
            builder.ExportOutput(port, f"{camera_name}.{port.get_name()}")

    ApplyVisualizationConfig(
        scenario.visualization, builder, lcm_buses, meshcat=meshcat
    )

    # Export "cheat" ports.
    builder.ExportOutput(scene_graph.get_query_output_port(), "query_object")
    builder.ExportOutput(sim_plant.get_contact_results_output_port(), "contact_results")
    builder.ExportOutput(sim_plant.get_state_output_port(), "plant_continuous_state")
    builder.ExportOutput(sim_plant.get_body_poses_output_port(), "body_poses")

    return builder.Build()


def MakeHardwareStationInterface(scenario: Scenario, meshcat: Meshcat):
    raise RuntimeError("Hardware interface not impl")
    builder = DiagramBuilder()

    diagram = builder.Build()
    diagram.set_name("HardwareStationInterface")
    return diagram
