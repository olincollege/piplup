import numpy as np
import matplotlib.pyplot as plt
from pydrake.all import *
from pydrake.geometry import (
    Meshcat,
    MeshcatVisualizer,
    SceneGraph,
    MeshcatPointCloudVisualizer,
)

import yaml
import os

from perception import ImageSegmenter


#Refactor to add coordinates to scenarios and pass into here (maybe in CameraInfo or a subclass)
def MakeCameraSegmenters(
    camera_info: {str: CameraInfo} = None,
    meshcat: Meshcat = None,
    hardware: bool = False,
) -> Diagram:
    builder = DiagramBuilder()

    file_path = 'perception_configs.yaml'

    file_path = os.path.join("/home/piplup/piplup/perception/", file_path)

    with open(file_path, 'r') as file:
        parsed_yaml = yaml.safe_load(file)

    segmenters = []

    for camera in camera_info.keys():
        segmenters.append(builder.AddNamedSystem(
            f"{camera}_seg", ImageSegmenter(camera, parsed_yaml[camera]["top_left_coordinate"],
                                             parsed_yaml[camera]["bottom_right_coordinate"], parsed_yaml[camera]["object_points"],
                                             parsed_yaml[camera]["background_points"])
        ))

        builder.ExportInput(segmenters[-1].depth_image_input_port, f"{camera}_depth_image")
        builder.ExportInput(segmenters[-1].color_image_input_port, f"{camera}_color_image")
        
        # Export the masked depth image output port of the segmenter to the diagram
        builder.ExportOutput(segmenters[-1].masked_depth_image, f"{camera}_masked_depth_image")
    
    return builder.Build()