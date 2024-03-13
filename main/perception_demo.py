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

from perception import MakePointCloudGenerator, ImageSegmenter
import cv2

from common.utils import PoseTransform
from planning import SuctionGraspSelector

from common.logging import *


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

    suction_grasp: SuctionGraspSelector = builder.AddNamedSystem(
        "suction_grasp_selector", SuctionGraspSelector(meshcat)
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
            f"{camera}_segmenter", ImageSegmenter(camera)
        )

        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.depth_image_16u"),
            seg.GetInputPort(f"{camera}_depth_image"),
        )

        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.color_image"),
            seg.GetInputPort(f"{camera}_color_image"),
        )

        builder.Connect(
            camera_pose.GetOutputPort("pose"),
            point_cloud_generator.GetInputPort(f"{camera}_pose"),
        )
        builder.Connect(
            seg.GetOutputPort(f"{camera}_masked_depth_image"),
            point_cloud_generator.GetInputPort(f"{camera}_depth_image"),
        )

    builder.Connect(
        point_cloud_generator.GetOutputPort("merged_point_cloud"),
        suction_grasp.GetInputPort("merged_point_cloud"),
    )
    # Build the diagram and its simulator.
    diagram: Diagram = builder.Build()

    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    simulator.Initialize()

    # from graphviz import Source
    # s = Source(diagram.GetGraphvizString(), filename="test.gv", format="png")
    # s.view()

    # Simulate.
    try:
        while True:
            simulator.AdvanceTo(simulator.get_context().get_time() + 0.5)
            # suction_grasp.GetOutputPort("grasp_selection").Eval(
            #     suction_grasp.GetMyContextFromRoot(simulator.get_context())
            # )
            if visualize:
                for camera in cameras:
                    if camera == "camera3":
                        img_color = (
                            hardware_station.GetOutputPort(f"{camera}.color_image")
                            .Eval(
                                hardware_station.GetMyContextFromRoot(
                                    simulator.get_context()
                                )
                            )
                            .data
                        )
                        img_depth = (
                            hardware_station.GetOutputPort(f"{camera}.depth_image_16u")
                            .Eval(
                                hardware_station.GetMyContextFromRoot(
                                    simulator.get_context()
                                )
                            )
                            .data
                        )
                        masked_color = (
                            diagram.GetSubsystemByName(f"{camera}_segmenter")
                            .GetOutputPort(f"{camera}_masked_color_image")
                            .Eval(
                                diagram.GetSubsystemByName(
                                    f"{camera}_segmenter"
                                ).GetMyContextFromRoot(simulator.get_context())
                            )
                            .data
                        )

                        masked_depth = (
                            diagram.GetSubsystemByName(f"{camera}_segmenter")
                            .GetOutputPort(f"{camera}_masked_depth_image")
                            .Eval(
                                diagram.GetSubsystemByName(
                                    f"{camera}_segmenter"
                                ).GetMyContextFromRoot(simulator.get_context())
                            )
                            .data
                        )

                        if img_color.size > 0 and img_depth.size > 0:
                            img_color = cv2.cvtColor(img_color, cv2.COLOR_RGB2BGR)
                            masked_color = cv2.cvtColor(masked_color, cv2.COLOR_RGB2BGR)
                            brightness = 20
                            contrast = 5
                            img_depth = cv2.cvtColor(img_depth, cv2.COLOR_GRAY2BGR)
                            img_depth = cv2.addWeighted(
                                img_depth,
                                contrast,
                                np.zeros(img_depth.shape, img_depth.dtype),
                                0,
                                brightness,
                            )
                            masked_depth = cv2.cvtColor(
                                masked_depth, cv2.COLOR_GRAY2BGR
                            )
                            masked_depth = cv2.addWeighted(
                                masked_depth,
                                contrast,
                                np.zeros(masked_depth.shape, masked_depth.dtype),
                                0,
                                brightness,
                            )
                            img_comb_c = cv2.hconcat([img_color, masked_color])
                            img_comb_d = cv2.hconcat([img_depth, masked_depth])
                            cv2.imshow(f"{camera}_all", img_comb_c)
                            cv2.imshow(f"{camera}_d", img_comb_d)

                if cv2.waitKey(1) == ord("q"):
                    break
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        logging.info(simulator.get_actual_realtime_rate())
        if hardware_station.HasSubsystemNamed("gen3_interface"):
            hardware_station.GetSubsystemByName("gen3_interface").CleanUp()


def main():
    init_logging()
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("-v", action="store_true")
    args = parser.parse_args()
    scenario_name = "Simulated" if args.sim else "Hardware"
    scenario = load_scenario(
        filename="models/perception_scenarios.yaml",
        scenario_name=scenario_name,
    )
    logging.info(f"Running {scenario_name} Perception Demo")
    run(scenario=scenario, visualize=args.v)


if __name__ == "__main__":
    main()
