import argparse
from pydrake.all import *
from pydrake.geometry import Meshcat, StartMeshcat
from pydrake.systems.analysis import ApplySimulatorConfig, Simulator
from pydrake.systems.framework import DiagramBuilder

from station import (
    MakeHardwareStation,
    Scenario,
    load_scenario,
    QuestTwistTeleopController,
)

from planning import OreoPlanner

from common.logging import *


def run(*, scenario: Scenario):
    meshcat: Meshcat = StartMeshcat()
    builder = DiagramBuilder()

    oreo_planner: OreoPlanner = builder.AddNamedSystem(
        "oreo_planner", OreoPlanner()
    )

    hardware_station: Diagram = builder.AddNamedSystem(
        "hardware_station", MakeHardwareStation(scenario, meshcat)
    )

    builder.Connect(
        oreo_planner.GetOutputPort("control_mode"),
        hardware_station.GetInputPort("gen3.control_mode"),
    )
    builder.Connect(
        oreo_planner.GetOutputPort("arm_command"),
        hardware_station.GetInputPort("gen3.command"),
    )
    gripper_name = scenario.model_drivers["gen3"].hand_model_name
    builder.Connect(
        oreo_planner.GetOutputPort("gripper_command"),
        hardware_station.GetInputPort(f"{gripper_name}.command"),
    )
    builder.Connect(
        hardware_station.GetOutputPort("gen3.pose_measured"),
        oreo_planner.GetInputPort("pose_measured"),
    )

    # gamepad: QuestTwistTeleopController = builder.AddNamedSystem(
    #     "quest_controller",
    #     QuestTwistTeleopController(meshcat, gripper_name),
    # )
    # builder.Connect(
    #     gamepad.GetOutputPort("V_WE_desired"),
    #     oreo_planner.GetInputPort("vr_command"),
    # )
    # builder.Connect(
    #     hardware_station.GetOutputPort("gen3.pose_measured"),
    #     gamepad.GetInputPort("pose"),
    # )
    # builder.Connect(
    #     gamepad.GetOutputPort(f"gripper_command"),
    #     oreo_planner.GetInputPort(f"vr_gripper"),
    # )
    # builder.Connect(
    #     gamepad.GetOutputPort(f"buttons"),
    #     oreo_planner.GetInputPort(f"vr_buttons"),
    # )

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
        logging.info(simulator.get_actual_realtime_rate())
        hardware_station.GetSubsystemByName("gen3_interface").CleanUp()


def main():
    init_logging()
    scenario = load_scenario(
        filename="models/oreo_scenario.yaml",
        scenario_name="OreoDemo",
    )
    logging.info(f"Running Oreo Demo")
    run(scenario=scenario)


if __name__ == "__main__":
    main()
