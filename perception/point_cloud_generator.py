"""
Systems for processing RealSense RGBD data.
"""

import numpy as np
import matplotlib.pyplot as plt
from pydrake.all import *
from pydrake.geometry import Meshcat, MeshcatPointCloudVisualizer


def MakePointCloudGenerator(
    camera_info: dict[str, CameraInfo],
    meshcat: Meshcat = None,
    hardware: bool = False,
) -> Diagram:
    # Create diagram
    builder = DiagramBuilder()

    # Add point cloud processor LeafSystem
    point_cloud_processor: PointCloudProcessor = builder.AddNamedSystem(
        "point_cloud_processor",
        PointCloudProcessor(cameras=list(camera_info.keys()), meshcat=meshcat),
    )

    # Add DepthImageToPointCloud LeafSystem for each camera
    for camera in camera_info.keys():
        image_to_point_cloud: DepthImageToPointCloud = builder.AddNamedSystem(
            f"image_to_point_cloud_{camera}",
            DepthImageToPointCloud(
                camera_info=camera_info[camera],
                pixel_type=PixelType.kDepth16U,
                scale=1.0
                / 1000.0,  # 16 bit depth is in millimeters not meters :( - Krishna
                fields=BaseField.kXYZs | BaseField.kNormals,
            ),
        )
        builder.ExportInput(
            image_to_point_cloud.camera_pose_input_port(), f"{camera}_pose"
        )
        builder.ExportInput(
            image_to_point_cloud.depth_image_input_port(), f"{camera}_depth_image"
        )
        builder.Connect(
            image_to_point_cloud.GetOutputPort("point_cloud"),
            point_cloud_processor.GetInputPort(f"{camera}_cloud"),
        )
        builder.ConnectInput(
            f"{camera}_pose",
            point_cloud_processor.GetInputPort(f"{camera}.pose"),
        )

        builder.ExportOutput(
            image_to_point_cloud.GetOutputPort("point_cloud"), f"{camera}.point_cloud"
        )

    # Add point cloud visualizer
    meshcat_point_cloud: MeshcatPointCloudVisualizer = builder.AddNamedSystem(
        "point_cloud_visualizer",
        MeshcatPointCloudVisualizer(
            meshcat=meshcat, path="point_cloud", publish_period=1.0
        ),
    )
    meshcat_point_cloud.set_point_size(0.002)
    # meshcat_point_cloud.set_default_rgba(Rgba(74.0/255.0, 140.0/255.0, 1.0))

    builder.Connect(
        point_cloud_processor.GetOutputPort("merged_point_cloud"),
        meshcat_point_cloud.GetInputPort("cloud"),
    )
    builder.ExportOutput(
        point_cloud_processor.GetOutputPort("merged_point_cloud"), "merged_point_cloud"
    )

    return builder.Build()


class PointCloudProcessor(LeafSystem):
    def __init__(self, cameras: list[str], meshcat: Meshcat = None, color=True):
        super().__init__()
        self.cameras = cameras
        self._meshcat = meshcat
        self._cloud_inputs: list[InputPort] = []
        self._cam_poses: list[InputPort] = []
        self.color = color
        # Add input port for each camera
        for camera in self.cameras:
            self._cloud_inputs.append(
                self.DeclareAbstractInputPort(
                    name=f"{camera}_cloud", model_value=AbstractValue.Make(PointCloud())
                )
            )
            self._cam_poses.append(
                self.DeclareAbstractInputPort(
                    f"{camera}.pose", AbstractValue.Make(RigidTransform())
                )
            )

        self.DeclareAbstractOutputPort(
            "merged_point_cloud",
            lambda: AbstractValue.Make(PointCloud()),
            self.UpdatePointCloud,
        )

        self.crop_lower = np.array([0.1, -0.3, 0])
        self.crop_upper = np.array([1.0, 0.3, 1.0])

    def UpdatePointCloud(self, context: Context, output: AbstractValue):
        clouds: list[PointCloud] = []

        # Evaluate point cloud input ports
        for i, (cloud_port, pose_port, camera) in enumerate(
            zip(self._cloud_inputs, self._cam_poses, self.cameras)
        ):
            pc = cloud_port.Eval(context)

            clouds.append(pc)
            clouds[i].EstimateNormals(radius=0.1, num_closest=30)
            X_WC = pose_port.Eval(context)
            clouds[i].FlipNormalsTowardPoint(X_WC.translation())
        # Merge clouds
        output.set_value(
            Concatenate(clouds=clouds)
            .VoxelizedDownSample(voxel_size=0.005)
            .Crop(self.crop_lower, self.crop_upper)
        )
        # self._meshcat.SetObject(
        #     f"/pointcloud",
        #     output,
        #     point_size=0.003,
        #     # rgba=Rgba((i / len(self.cameras)), 0, 0, 1),
        #     # rgba=Rgba(1,0,0,1) if self.color else Rgba(0,1,0,1),
        # )
        # Save point cloud for perception system testing
        # np.save('/home/ali1/code/piplup/test_data/point_cloud.npy', merged_cloud.xyzs())

        # self._meshcat.SetLineSegments(
        #     "normals",
        #     merged_cloud.xyzs(),
        #     merged_cloud.xyzs() + 0.01 * merged_cloud.normals(),
        # )
