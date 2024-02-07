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
from image_segmenter import ImageSegmenter


#Refactor to add coordinates to scenarios and pass into here (maybe in CameraInfo or a subclass)
def MakeCameraSegmenters(
    camera_info: {str: CameraInfo} = None,
    meshcat: Meshcat = None,
    hardware: bool = False,
) -> Diagram:
    builder = DiagramBuilder()

    file_path = 'perception_configs.yaml'

    print("1")
    # Open the YAML file and load its content
    with open(file_path, 'r') as file:
        print("2")
        parsed_yaml = yaml.safe_load(file)

    segmenters = []

    for camera in camera_info.keys():
        segmenters.append(builder.AddNamedSystem(
            f"{camera}_seg", ImageSegmenter(camera, parsed_yaml[camera]["top_left_coordinate"],
                                             parsed_yaml[camera]["top_left_coordinate"], parsed_yaml[camera]["object_coordinates"],
                                             parsed_yaml[camera]["background_coordinates"])
        ))
    
    return builder