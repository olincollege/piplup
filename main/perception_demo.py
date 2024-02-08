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

from perception import MakePointCloudGenerator

import cv2


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
            hardware_station.GetInputPort("gen3.twist"),
        )
        builder.Connect(
            hardware_station.GetOutputPort("gen3.pose_measured"),
            gamepad.GetInputPort("pose"),
        )

    camera_info: {str: CameraInfo} = {}
    cameras = list(scenario.cameras.keys())

    for camera in cameras:
        camera_info[camera] = hardware_station.GetSubsystemByName(
            f"rgbd_sensor_{camera}"
        ).depth_camera_info()

    point_cloud_generator: Diagram = builder.AddNamedSystem(
        "point_cloud_generator",
        MakePointCloudGenerator(camera_info=camera_info, meshcat=meshcat),
    )

    for camera in cameras:
        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.body_pose_in_world"),
            point_cloud_generator.GetInputPort(f"{camera}_pose"),
        )

        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.depth_image_16u"),
            point_cloud_generator.GetInputPort(f"{camera}_depth_image"),
        )

    # Build the diagram and its simulator.
    diagram: Diagram = builder.Build()

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
        plot_system_graphviz(diagram, options=options)
        plt.show()

    simulator.Initialize()
    # Simulate.

    while True:
        try:
            # simulator.AdvanceTo(scenario.simulation_duration)
            simulator.AdvanceTo(simulator.get_context().get_time() + 0.05)

            for camera in cameras:
                img_color = (
                    hardware_station.GetOutputPort(f"{camera}.color_image")
                    .Eval(
                        hardware_station.GetMyContextFromRoot(simulator.get_context())
                    )
                    .data
                )
                img_depth = (
                    hardware_station.GetOutputPort(f"{camera}.depth_image_16u")
                    .Eval(
                        hardware_station.GetMyContextFromRoot(simulator.get_context())
                    )
                    .data
                )
                if img_color.size > 0 and img_depth.size > 0:
                    img_color = cv2.cvtColor(img_color, cv2.COLOR_RGB2BGR)
                    cv2.imshow(f"{camera}_c", img_color)
                    cv2.imshow(f"{camera}_d", img_depth)

            if cv2.waitKey(1) == ord("q"):
                break
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            print(simulator.get_actual_realtime_rate())
            if hardware_station.HasSubsystemNamed("gen3_interface"):
                hardware_station.GetSubsystemByName("gen3_interface").CleanUp()
            break


def main():
    scenario = load_scenario(
        filename="models/teleop_scenarios_hardware.yaml",
        scenario_name="TeleopSuctionGripper",
    )
    run(scenario=scenario, graphviz=None)


if __name__ == "__main__":
    main()
