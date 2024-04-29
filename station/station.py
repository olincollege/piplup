from pydrake.all import *
from pydrake.geometry import Meshcat
from pydrake.manipulation._manipulation_extra import ApplyDriverConfigs
from pydrake.multibody.parsing import ModelDirectives, ProcessModelDirectives
from pydrake.multibody.plant import AddMultibodyPlant
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import ApplyCameraConfig
from pydrake.visualization import ApplyVisualizationConfig
from realsensed400 import RealSenseD400, RealsenseInterfaceConfig  # type: ignore
from kinova_gen3 import Gen3InterfaceConfig, Gen3HardwareInterface, Gen3ControlMode
from uss_dbs import USSDBSHardwareInterface, USSDBSInterfaceConfig
from common import ConfigureParser
from station.scenario import Scenario
from typing import Any, ClassVar, List, Optional
from epick_interface import EPickInterfaceConfig, EPickInterface, ObjectDetectionStatus  # type: ignore

import numpy as np

from common.logging import *


def MakeHardwareStation(
    scenario: Scenario,
    meshcat: Meshcat = None,
) -> Diagram:
    builder = DiagramBuilder()

    logging.info(f"Creating Hardware Station")
    if scenario.hardware_interface:
        scenario.plant_config.time_step = 0.0005
        scenario.visualization.publish_period = 0.0005

    # Create the multibody plant and scene graph.
    logging.debug(f"Adding Multibody Plant")
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

    if scenario.hardware_interface:
        logging.info(f"Hardware interface exists in scenario, creating interface")
        # Set the actuation of the sim plant to 0
        zero_actuation_sys = builder.AddSystem(ConstantVectorSource(np.zeros(9)))
        builder.Connect(
            zero_actuation_sys.get_output_port(0), sim_plant.get_actuation_input_port()
        )

        return MakeHardwareStationInterface(builder, scenario, meshcat, sim_plant)

    lcm_buses = None

    logging.debug(f"Applying Driver Configs")
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
        logging.debug(f"Adding Simulated Camera: {camera_name}")
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

    logging.info(f"Finished Creating Hardware Station")
    return builder.Build()


def MakeHardwareStationInterface(
    builder: DiagramBuilder,
    scenario: Scenario,
    meshcat: Meshcat,
    sim_plant: MultibodyPlant,
):
    # This is not currently using a config since we are only using one scale
    # uss_dbs_system: System = builder.AddNamedSystem(
    #     "uss_dbs", USSDBSHardwareInterface()
    # )
    # builder.ExportOutput(uss_dbs_system.GetOutputPort("mass"), "uss_dbs.mass")

    for model_name, hardware_interface in scenario.hardware_interface.items():
        interface_subsystem = None
        if isinstance(hardware_interface, RealsenseInterfaceConfig):
            hardware_interface: RealsenseInterfaceConfig
            camera_config = scenario.cameras.get(model_name)
            if not camera_config:
                raise RuntimeError(
                    f"Failed to create hardware interface for camera model: {model_name}, check if camera exists in scenario->camera"
                )

            interface_subsystem: RealSenseD400 = builder.AddNamedSystem(
                f"rgbd_sensor_{model_name}",
                RealSenseD400(
                    hardware_interface.serial_number, camera_config, sim_plant
                ),
            )
        elif isinstance(hardware_interface, Gen3InterfaceConfig):
            model_driver = scenario.model_drivers[model_name]
            gen3_model = model_driver
            interface_subsystem = builder.AddNamedSystem(
                "gen3_interface",
                Gen3HardwareInterface(
                    hardware_interface.ip_address,
                    hardware_interface.port,
                    gen3_model.hand_model_name,
                    sim_plant,
                ),
            )
        elif isinstance(hardware_interface, EPickInterfaceConfig):
            interface_subsystem = builder.AddNamedSystem(
                "epick", EPickInterface(hardware_interface)
            )
        else:
            raise RuntimeError(
                f"Invalid hardware interface type {hardware_interface} for model {model_name}"
            )

        for i in range(interface_subsystem.num_input_ports()):
            port = interface_subsystem.get_input_port(i)
            if "2f_85" in port.get_name():
                builder.ExportInput(port, port.get_name())
            if not builder.IsConnectedOrExported(port):
                builder.ExportInput(port, f"{model_name}.{port.get_name()}")
        for i in range(interface_subsystem.num_output_ports()):
            port = interface_subsystem.get_output_port(i)
            builder.ExportOutput(port, f"{model_name}.{port.get_name()}")

    if builder.empty():
        raise RuntimeError("No systems were added to hardware interface")

    ApplyVisualizationConfig(scenario.visualization, builder, None, meshcat=meshcat)
    diagram: Diagram = builder.Build()
    diagram.set_name("HardwareStationInterface")
    return diagram
