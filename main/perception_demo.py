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


class ImageSegmentationSystem(LeafSystem):
    def __init__(self, label):
        LeafSystem.__init__(self)
        self.label = label

        self.label_img_port = self.DeclareAbstractInputPort(
            "label_image", AbstractValue.Make(ImageLabel16I())
        )
        self.depth_img_port = self.DeclareAbstractInputPort(
            "depth_image", AbstractValue.Make(ImageDepth16U())
        )
        self.DeclareAbstractOutputPort(
            "masked_image", lambda: AbstractValue.Make(ImageDepth16U()), self.MaskImage
        )

    def MaskImage(self, context: Context, output: AbstractValue):
        label_img = self.label_img_port.Eval(context).data
        depth_img = self.depth_img_port.Eval(context).data

        mask_img = label_img == self.label
        depth_img = copy.copy(depth_img)
        depth_img[~mask_img] = 0

        img: ImageDepth16U = output.get_mutable_value()
        img.resize(640, 480)
        img.mutable_data[:] = depth_img


class PoseTransform(LeafSystem):
    def __init__(
        self,
        X_BA: RigidTransform = RigidTransform(),
    ):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort("pose", AbstractValue.Make(RigidTransform()))
        self.DeclareAbstractOutputPort(
            "pose",
            lambda: AbstractValue.Make(RigidTransform()),
            self._CalcOutput,
        )
        self.X_BA = X_BA

    def _CalcOutput(self, context, output):
        pose = self.EvalAbstractInput(context, 0).get_value()
        pose = pose @ self.X_BA
        output.get_mutable_value().set(pose.rotation(), pose.translation())


def AddMeshcatTriad(
    meshcat: Meshcat,
    path: str,
    length: float = 0.25,
    radius: float = 0.01,
    opacity: float = 1.0,
    X_PT: RigidTransform = RigidTransform(),
):
    """Adds an X-Y-Z triad to the meshcat scene.

    Args:
        meshcat: A Meshcat instance.
        path: The Meshcat path on which to attach the triad. Using relative paths will attach the triad to the path's coordinate system.
        length: The length of the axes in meters.
        radius: The radius of the axes in meters.
        opacity: The opacity of the axes in [0, 1].
        X_PT: The pose of the triad relative to the path.
    """
    meshcat.SetTransform(path, X_PT)
    # x-axis
    X_TG = RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2), [length / 2.0, 0, 0])
    meshcat.SetTransform(path + "/x-axis", X_TG)
    meshcat.SetObject(
        path + "/x-axis", Cylinder(radius, length), Rgba(1, 0, 0, opacity)
    )

    # y-axis
    X_TG = RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2), [0, length / 2.0, 0])
    meshcat.SetTransform(path + "/y-axis", X_TG)
    meshcat.SetObject(
        path + "/y-axis", Cylinder(radius, length), Rgba(0, 1, 0, opacity)
    )

    # z-axis
    X_TG = RigidTransform([0, 0, length / 2.0])
    meshcat.SetTransform(path + "/z-axis", X_TG)
    meshcat.SetObject(
        path + "/z-axis", Cylinder(radius, length), Rgba(0, 0, 1, opacity)
    )


class SuctionGraspSelector(LeafSystem):
    def __init__(self, meshcat):
        LeafSystem.__init__(self)

        self.point_cloud_port_ = self.DeclareAbstractInputPort(
            "merged_point_cloud", AbstractValue.Make(PointCloud())
        )
        self.meshcat: Meshcat = meshcat
        self.DeclareAbstractOutputPort(
            "grasp_selection",
            lambda: AbstractValue.Make((np.inf, RigidTransform())),
            self.GenerateGrasp,
        )

        self._rng = np.random.default_rng()
        self.cup_diam = 0.04
        self.cup_depth = 0.005
        self.meshcat.SetObject(
            "/cropping_box",
            Cylinder(self.cup_diam / 2, self.cup_depth),
            rgba=Rgba(0, 1, 0, 0.25),
        )

        self.X_end_to_tool_inv = RigidTransform(
            AngleAxis(np.pi, np.array([1, 0, 0])), np.array([0, 0, 0.06])
        )

    def GenerateGrasp(self, context: Context, output: AbstractValue):
        pc: PointCloud = self.point_cloud_port_.Eval(context)

        centroid = np.mean(pc.xyzs(), axis=1)
        AddMeshcatTriad(
            self.meshcat,
            "/centroid",
            length=0.05,
            radius=0.003,
            X_PT=RigidTransform(centroid),
        )

        candidates = []
        scores = []
        crops = []
        for i in range(300):
            index = self._rng.integers(0, pc.size() - 1)
            p_WS = pc.xyz(index)
            n_WS = pc.normal(index)
            if np.abs(np.dot(np.array([0.0, 0.0, -1.0]), n_WS)) < 1e-6:
                continue

            # Crop to surrounding region
            Gy = np.cross(n_WS, np.array([0, 0, 1]))
            Gy = Gy / np.linalg.norm(Gy)  # normalize
            Gx = np.cross(Gy, n_WS)
            R_WG = RotationMatrix(np.vstack((Gx, Gy, n_WS)).T)
            lower = R_WG @ (
                -np.array([self.cup_diam, self.cup_diam, self.cup_depth]) / 2
            )
            upper = R_WG @ (
                np.array([self.cup_diam, self.cup_diam, self.cup_depth]) / 2
            )
            p1 = np.minimum(lower, upper)
            p2 = np.maximum(lower, upper)
            print(p1)
            print(p2)
            if np.all(p1 > p2) or np.any(np.isnan(p1)) or np.any(np.isnan(p2)):
                continue
            cropped = pc.Crop(p_WS + p1, p_WS + p2)

            if (
                cropped.size() > 70
                and np.abs(np.dot(np.array([0.0, 0.0, 1.0]), n_WS)) > 0.8
            ):
                score = p_WS[2] + 1 / np.linalg.norm(p_WS[:2] - centroid[:2])
                scores.append(score)
                candidates.append(RigidTransform(R_WG, p_WS))
                crops.append(cropped)

        if scores:
            i = np.array(scores).argmax()
            G = candidates[i]
            cropped = crops[i]

            self.meshcat.SetObject(
                "/cropped", cropped, point_size=0.003, rgba=Rgba(0, 1, 0, 1)
            )
            self.meshcat.SetTransform("/cropping_box", G)
            AddMeshcatTriad(
                self.meshcat, "/pick_point", length=0.02, radius=0.001, X_PT=G
            )
            AddMeshcatTriad(
                self.meshcat,
                "/ee_pose",
                length=0.02,
                radius=0.001,
                X_PT=G @ self.X_end_to_tool_inv,
            )
            # X_WT = X_WE * X_ET
            output.set_value((scores[i], G @ self.X_end_to_tool_inv))
        else:
            output.set_value((0, RigidTransform()))


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

    # suction_grasp: SuctionGraspSelector = builder.AddNamedSystem(
    #     "suction_grasp_selector", SuctionGraspSelector(meshcat)
    # )

    for camera in cameras:
        camera_pose = builder.AddNamedSystem(
            f"{camera}.pose",
            PoseTransform(scenario.cameras[camera].X_BD.GetDeterministicValue()),
        )
        builder.Connect(
            hardware_station.GetOutputPort(f"{camera}.body_pose_in_world"),
            camera_pose.GetInputPort("pose"),
        )
        builder.Connect(
            camera_pose.GetOutputPort("pose"),
            point_cloud_generator.GetInputPort(f"{camera}_pose"),
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
            seg.GetOutputPort(f"{camera}_masked_depth_image"),
            point_cloud_generator.GetInputPort(f"{camera}_depth_image"),
        )

        # seg: System = builder.AddNamedSystem(
        #     f"{camera}_segmenter", ImageSegmentationSystem(label=8)
        # )

        # builder.Connect(
        #     hardware_station.GetOutputPort(f"{camera}.label_image"),
        #     seg.GetInputPort("label_image"),
        # )
        # builder.Connect(
        #     hardware_station.GetOutputPort(f"{camera}.depth_image_16u"),
        #     seg.GetInputPort("depth_image"),
        # )

        # builder.Connect(
        #     seg.GetOutputPort("masked_image"),
        #     point_cloud_generator.GetInputPort(f"{camera}_depth_image"),
        # )

    # builder.Connect(
    #     point_cloud_generator.GetOutputPort("merged_point_cloud"),
    #     suction_grasp.GetInputPort("merged_point_cloud"),
    # )
    # Build the diagram and its simulator.
    diagram: Diagram = builder.Build()

    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    simulator.Initialize()

    # from graphviz import Source
    # s = Source(diagram.GetGraphvizString(), filename="test.gv", format="png")
    # s.view()

    # Simulate.
    while True:
        try:
            simulator.AdvanceTo(simulator.get_context().get_time() + 0.05)
            # suction_grasp.GetOutputPort("grasp_selection").Eval(
            #     suction_grasp.GetMyContextFromRoot(simulator.get_context())
            # )
            if visualize:
                for camera in cameras:
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
