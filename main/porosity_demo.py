import argparse
from pydrake.all import *
from pydrake.geometry import Meshcat, StartMeshcat
from pydrake.systems.analysis import ApplySimulatorConfig, Simulator
from pydrake.systems.framework import DiagramBuilder

from station import (
    MakeHardwareStation,
    Scenario,
    load_scenario,
)

from planning import PorosityPlanner


def run(*, scenario: Scenario):
    meshcat: Meshcat = StartMeshcat()
    builder = DiagramBuilder()

    porosity_planner: PorosityPlanner = builder.AddNamedSystem(
        "porosity_planner", PorosityPlanner()
    )

    hardware_station: Diagram = builder.AddNamedSystem(
        "hardware_station", MakeHardwareStation(scenario, meshcat)
    )

    builder.Connect(
        porosity_planner.GetOutputPort("control_mode"),
        hardware_station.GetInputPort("gen3.control_mode"),
    )
    builder.Connect(
        porosity_planner.GetOutputPort("arm_command"),
        hardware_station.GetInputPort("gen3.command"),
    )
    builder.Connect(
        hardware_station.GetOutputPort("epick.actual_vacuum_pressure"),
        porosity_planner.GetInputPort("actual_vacuum_pressure"),
    )
    builder.Connect(
        porosity_planner.GetOutputPort("suction_command"),
        hardware_station.GetInputPort("epick.command"),
    )
    builder.Connect(
        hardware_station.GetOutputPort("epick.object_detection_status"),
        porosity_planner.GetInputPort("object_detection_status"),
    )
    builder.Connect(
        hardware_station.GetOutputPort("gen3.pose_measured"),
        porosity_planner.GetInputPort("pose_measured"),
    )

    diagram: Diagram = builder.Build()
    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)
    hardware_station.GetSubsystemByName("gen3_interface").root_ctx = (
        simulator.get_mutable_context()
    )

    simulator.Initialize()

    try:
        simulator.AdvanceTo(scenario.simulation_duration)
    except KeyboardInterrupt:
        print(simulator.get_actual_realtime_rate())
        hardware_station.GetSubsystemByName("gen3_interface").CleanUp()
        hardware_station.GetSubsystemByName("epick").release()


def main():
    scenario = load_scenario(
        filename="models/porosity_scenario.yaml",
        scenario_name="PorosityDemo",
    )
    run(scenario=scenario)


if __name__ == "__main__":
    main()
