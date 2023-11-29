import argparse
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import *
from pydrake.common import RandomGenerator
from pydrake.geometry import Meshcat, StartMeshcat, MeshcatPointCloudVisualizer
from pydrake.systems.analysis import ApplySimulatorConfig, Simulator
from pydrake.systems.framework import DiagramBuilder

from kinova_gen3 import GamepadDiffIkController
from station import MakeHardwareStation, Scenario, load_scenario
from perception import MakePointCloudGenerator


def run(*, scenario: Scenario, graphviz=None):
    meshcat: Meshcat = StartMeshcat()
    builder = DiagramBuilder()
    hardware_station: Diagram = builder.AddNamedSystem(
        "hardware_station", MakeHardwareStation(scenario, meshcat)
    )
    

    # TODO (krishna) This should be more generic
    # ----------
    gripper_name = scenario.model_drivers["gen3"].hand_model_name
    controller_plant: MultibodyPlant = hardware_station.GetSubsystemByName(
        "gen3_controller_plant"
    ).get()
    # ----------

    gamepad: GamepadDiffIkController = builder.AddNamedSystem(
        "gamepad_control",
        GamepadDiffIkController(meshcat, controller_plant, gripper_name),
    )

    camera_info: {str: CameraInfo} = {}
    cameras = list(scenario.cameras.keys())

    for camera in cameras:
        camera_info[camera] = hardware_station.GetSubsystemByName(f"rgbd_sensor_{camera}").depth_camera_info()

    point_cloud_generator: Diagram = builder.AddNamedSystem(
        "point_cloud_generator", MakePointCloudGenerator(camera_info=camera_info, meshcat=meshcat)
    )
    
    for camera in cameras:
        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.body_pose_in_world"),
            point_cloud_generator.GetInputPort(f"{camera}_pose")
        )

        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.depth_image_32f"),
            point_cloud_generator.GetInputPort(f"{camera}_depth_image")
        )

    builder.Connect(
        gamepad.GetOutputPort("gen3.position"),
        hardware_station.GetInputPort("gen3.position"),
    )
    builder.Connect(
        gamepad.GetOutputPort(f"{gripper_name}.command"),
        hardware_station.GetInputPort(f"{gripper_name}.command"),
    )

    builder.Connect(
        hardware_station.GetOutputPort("gen3.state_estimated"),
        gamepad.GetInputPort("gen3.state"),
    )

    # Build the diagram and its simulator.
    diagram: Diagram = builder.Build()

    meshcat.SetObject("ee_sphere", Sphere(0.05), Rgba(0, 0.5, 0, 0.5))
    ee_base = Mesh(
        "models/robotiq_description/meshes/visual/robotiq_arg2f_85_base_link.obj", 1
    )
    meshcat.SetObject("ee_body", ee_base, Rgba(0, 0.5, 0, 0.5))
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
        plot_system_graphviz(diagram, options=options)
        plt.show()

    camera_info: {str: CameraInfo} = {}
    cameras = list(scenario.cameras.keys())

    # for camera in cameras:
    img_color = hardware_station.GetOutputPort("camera0.color_image").Eval(hardware_station.CreateDefaultContext()).data
    f, axarr = plt.subplots(1,2) 
    axarr[0].imshow(img_color)
    img_depth = hardware_station.GetOutputPort("camera0.depth_image_32f").Eval(hardware_station.CreateDefaultContext()).data
    axarr[1].imshow(img_depth)
    plt.show()



    # Simulate.
    simulator.AdvanceTo(scenario.simulation_duration)


def main():
    parser = argparse.ArgumentParser(
        description="Run teleop demo for simulated hardware station"
    )
    parser.add_argument(
        "--scenario_name","-s",
        choices=["TeleopSuctionGripper", "TeleopPlanarGripper"],
        default="TeleopPlanarGripper",
    )
    args = parser.parse_args()
    scenario = load_scenario(
        filename="models/teleop_scenarios.yaml",
        scenario_name=args.scenario_name,
    )
    run(scenario=scenario, graphviz=None)


if __name__ == "__main__":
    main()
