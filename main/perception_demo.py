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


def run(*, scenario: Scenario, visualize=False):
    meshcat: Meshcat = StartMeshcat()
    builder = DiagramBuilder()
    hardware_station: Diagram = builder.AddNamedSystem(
        "hardware_station", MakeHardwareStation(scenario, meshcat)
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

    simulator.Initialize()
    # Simulate.

    while True:
        try:
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("-v", action="store_true")
    args = parser.parse_args()
    scenario = load_scenario(
        filename="models/perception_scenarios.yaml",
        scenario_name="Simulated" if args.sim else "Hardware",
    )
    run(scenario=scenario, visualize=args.v)


if __name__ == "__main__":
    main()
