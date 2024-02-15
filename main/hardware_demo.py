import argparse
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import *
from pydrake.common import RandomGenerator
from pydrake.geometry import Meshcat, StartMeshcat
from pydrake.systems.analysis import ApplySimulatorConfig, Simulator
from pydrake.systems.framework import DiagramBuilder

from station import (
    MakeHardwareStation,
    Scenario,
    load_scenario,
    GamepadTwistTeleopController,
    QuestTwistTeleopController,
)

from kinova_gen3 import Gen3ControlMode


def run(*, scenario: Scenario, graphviz=None, teleop=None):
    meshcat: Meshcat = StartMeshcat()
    builder = DiagramBuilder()
    hardware_station: Diagram = builder.AddNamedSystem(
        "hardware_station", MakeHardwareStation(scenario, meshcat)
    )

    gripper_name = scenario.model_drivers["gen3"].hand_model_name
    if teleop:
        gamepad: System = builder.AddNamedSystem(
            "gamepad_control",
            (
                GamepadTwistTeleopController(meshcat, gripper_name)
                if teleop == "gamepad"
                else QuestTwistTeleopController(meshcat, gripper_name)
            ),
        )

        builder.Connect(
            gamepad.GetOutputPort(f"gripper_command"),
            hardware_station.GetInputPort(f"{gripper_name}.command"),
        )

    if "gen3" in scenario.hardware_interface:
        builder.Connect(
            gamepad.GetOutputPort("V_WE_desired"),
            hardware_station.GetInputPort("gen3.command"),
        )
        builder.Connect(
            hardware_station.GetOutputPort("gen3.pose_measured"),
            gamepad.GetInputPort("pose"),
        )

    # Build the diagram and its simulator.
    diagram: Diagram = builder.Build()

    meshcat.SetObject("ee_sphere", Sphere(0.05), Rgba(0, 0.5, 0, 0.5))
    ee_base = Mesh(
        "models/robotiq_description/meshes/visual/robotiq_arg2f_85_base_link.obj", 1
    )
    meshcat.SetObject("ee_body", ee_base, Rgba(0, 0.5, 0, 0.5))
    meshcat.SetObject("test_body", ee_base, Rgba(0.5, 0.5, 0, 0.5))
    meshcat.SetCameraPose(np.array([1, -1, 1]) * 0.75, np.array([0, 0, 0.4]))
    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    # Sample the random elements of the context.
    random = RandomGenerator(scenario.random_seed)
    diagram.SetRandomContext(simulator.get_mutable_context(), random)

    # Visualize the diagram, when requested.
    options = {"plant/split": "I/O"}
    if graphviz:
        if isinstance(graphviz, str):
            with open(graphviz, "w", encoding="utf-8") as f:
                f.write(diagram.GetGraphvizString(options=options))

        plt.figure()
        plot_system_graphviz(diagram, options=options)
        plt.show()

    hardware_station.GetSubsystemByName("gen3_interface").root_ctx = (
        simulator.get_mutable_context()
    )

    hardware_station.GetInputPort(f"gen3.control_mode").FixValue(
        hardware_station.GetMyContextFromRoot(simulator.get_context()),
        Gen3ControlMode.kTwist,
    ),

    simulator.Initialize()

    try:
        simulator.AdvanceTo(scenario.simulation_duration)
    except KeyboardInterrupt:
        print(simulator.get_actual_realtime_rate())
        if hardware_station.HasSubsystemNamed("gen3_interface"):
            hardware_station.GetSubsystemByName("gen3_interface").CleanUp()


def main():
    parser = argparse.ArgumentParser(
        description="Run teleop demo for real hardware station"
    )
    parser.add_argument(
        "--scenarios_yaml",
        "-f",
        help="Scenarios YAML file path.",
        metavar="FILE",
        default="models/teleop_scenarios_hardware.yaml",
    )
    parser.add_argument(
        "--scenario_name",
        "-s",
        help="Specifies the scenario name within the scenario yaml file.",
        choices=["TeleopSuctionGripper", "TeleopPlanarGripper"],
        default="TeleopPlanarGripper",
    )
    parser.add_argument(
        "--teleop",
        "-t",
        help="If specified a Teleop interface will be enabled.",
        choices=[None, "gamepad", "quest"],
        default=None,
    )
    parser.add_argument("--graph_viz", "-g", action="store_true")
    args = parser.parse_args()
    scenario = load_scenario(
        filename=args.scenarios_yaml,
        scenario_name=args.scenario_name,
    )
    run(scenario=scenario, graphviz=args.graph_viz, teleop=args.teleop)


if __name__ == "__main__":
    main()
