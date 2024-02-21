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
)

from perception import MakePointCloudGenerator, LabelImageSegmentationSystem
import cv2

from piplup.utils import PoseTransform
from planning import PlanarGraspSelector
from kinova_gen3 import Gen3ControlMode


def run(*, scenario: Scenario, visualize=False):
    meshcat: Meshcat = StartMeshcat()
    builder = DiagramBuilder()
    hardware_station: Diagram = builder.AddNamedSystem(
        "hardware_station", MakeHardwareStation(scenario, meshcat)
    )

    camera_info: dict[str, CameraInfo] = {}
    cameras = list(scenario.cameras.keys())

    for camera in cameras:
        camera_info[camera] = hardware_station.GetSubsystemByName(
            f"rgbd_sensor_{camera}"
        ).depth_camera_info()

    point_cloud_generator: Diagram = builder.AddNamedSystem(
        "point_cloud_generator",
        MakePointCloudGenerator(camera_info=camera_info, meshcat=meshcat),
    )

    planar_grasp: PlanarGraspSelector = builder.AddNamedSystem(
        "planaer_grasp_selector", PlanarGraspSelector(meshcat)
    )

    for camera in cameras:
        camera_pose: PoseTransform = builder.AddNamedSystem(
            f"{camera}.pose",
            PoseTransform(scenario.cameras[camera].X_BD.GetDeterministicValue()),
        )
        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.body_pose_in_world"),
            camera_pose.GetInputPort("pose"),
        )

        seg: System = builder.AddNamedSystem(
            f"{camera}_segmenter", LabelImageSegmentationSystem(1)
        )

        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.depth_image_16u"),
            seg.GetInputPort(f"depth_image"),
        )

        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.label_image"),
            seg.GetInputPort(f"label_image"),
        )

        builder.Connect(
            camera_pose.GetOutputPort("pose"),
            point_cloud_generator.GetInputPort(f"{camera}_pose"),
        )
        builder.Connect(
            seg.GetOutputPort(f"masked_image"),
            point_cloud_generator.GetInputPort(f"{camera}_depth_image"),
        )

    builder.Connect(
        point_cloud_generator.GetOutputPort("merged_point_cloud"),
        planar_grasp.GetInputPort("merged_point_cloud"),
    )
    # Build the diagram and its simulator.
    diagram: Diagram = builder.Build()

    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    hardware_station_context = hardware_station.GetMyContextFromRoot(simulator.get_context())

    hardware_station.GetInputPort(f"gen3.control_mode").FixValue(
        hardware_station_context,
        Gen3ControlMode.kPosition,
    )
    hardware_station.GetInputPort(f"gen3.command").FixValue(
        hardware_station_context,
        [0, 0.56, 0, 1.02, 0, 1.29, 0],
    )
    hardware_station.GetInputPort(f"2f_85.command").FixValue(
        hardware_station_context,
        [0],
    )
    
    # label_image = hardware_station.GetOutputPort(f"camera0.label_image").Eval(
    #     hardware_station_context
    # ).data

    # plt.imshow(label_image)
    # plt.show()

    simulator.Initialize()

    # from graphviz import Source
    # s = Source(diagram.GetGraphvizString(), filename="test.gv", format="png")
    # s.view()

    # Simulate.
    try:
        while True:
            simulator.AdvanceTo(simulator.get_context().get_time() + 0.5)
            planar_grasp.GetOutputPort("grasp_selection").Eval(
                planar_grasp.GetMyContextFromRoot(simulator.get_context())
            )

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print(simulator.get_actual_realtime_rate())
        if hardware_station.HasSubsystemNamed("gen3_interface"):
            hardware_station.GetSubsystemByName("gen3_interface").CleanUp()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("-v", action="store_true")
    args = parser.parse_args()
    scenario = load_scenario(
        filename="models/grasp_planning_scenario.yaml",
        scenario_name="Simulated" if args.sim else "Hardware",
    )
    run(scenario=scenario, visualize=args.v)


if __name__ == "__main__":
    main()