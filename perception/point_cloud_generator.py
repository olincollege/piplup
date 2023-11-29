"""
Systems for processing RealSense RGBD data.
"""

import numpy as np
import matplotlib.pyplot as plt
from pydrake.all import *
from pydrake.geometry import (
    Meshcat,
    MeshcatVisualizer,
    SceneGraph,
    MeshcatPointCloudVisualizer
)
import pandas as pd

def MakePointCloudGenerator(
    camera_info: {str: CameraInfo},
    meshcat: Meshcat = None,
    hardware: bool = False,
) -> Diagram:
    builder = DiagramBuilder()

    point_cloud_processor: PointCloudProcessor = builder.AddNamedSystem(
        "point_cloud_processor", PointCloudProcessor(cameras=list(camera_info.keys()), meshcat=meshcat)
    )

    for camera in camera_info.keys():
        image_to_point_cloud: DepthImageToPointCloud = builder.AddNamedSystem(
            f"image_to_point_cloud_{camera}", DepthImageToPointCloud(camera_info=camera_info[camera])
            )
        builder.ExportInput(image_to_point_cloud.camera_pose_input_port(), f"{camera}_pose")
        builder.ExportInput(image_to_point_cloud.depth_image_input_port(), f"{camera}_depth_image")
        builder.Connect(
            image_to_point_cloud.GetOutputPort("point_cloud"),
            point_cloud_processor.GetInputPort(f"{camera}_cloud")
        )
    
    meshcat_point_cloud: MeshcatPointCloudVisualizer = builder.AddNamedSystem(
        "point_cloud_visualizer", MeshcatPointCloudVisualizer(meshcat=meshcat, path="point_cloud", publish_period=0.2)
    )

    builder.Connect(
        point_cloud_processor.GetOutputPort("merged_point_cloud"),
        meshcat_point_cloud.GetInputPort("cloud")
    )

    return builder.Build()

class PointCloudProcessor(LeafSystem):
    def __init__(self, cameras: [str], meshcat: Meshcat = None):
        super().__init__()
        self.cameras = cameras
        self._meshcat = meshcat
        self._cloud_inputs: [InputPort] = []

        for camera in self.cameras:
            self._cloud_inputs.append(self.DeclareAbstractInputPort(
                name=f"{camera}_cloud",
                model_value=AbstractValue.Make(PointCloud())
            ))
        
        self.merged_point_cloud_idx = self.DeclareAbstractState(
            AbstractValue.Make(PointCloud())
        )

        self.DeclarePeriodicDiscreteUpdateEvent(1/60.0, 0, self.UpdatePointCloud)

        self.DeclareStateOutputPort("merged_point_cloud", self.merged_point_cloud_idx)

    def UpdatePointCloud(self, context: Context, discrete_state: DiscreteValues):
        clouds: [PointCloud] = []

        for input in self._cloud_inputs:
            clouds.append(input.Eval(context))
            # clouds[-1].EstimateNormals(radius=0.1, num_closest=30)
        
        merged_cloud: PointCloud = Concatenate(clouds=clouds)

        # self._meshcat.SetLineSegments(
        #     "normals",
        #     merged_cloud.xyzs(),
        #     merged_cloud.xyzs() + 0.01 * merged_cloud.normals(),
        # )

        context.SetAbstractState(self.merged_point_cloud_idx, merged_cloud)
        