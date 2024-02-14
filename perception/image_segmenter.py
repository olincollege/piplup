import cv2
import numpy as np
from pydrake.all import *
from fastsam import FastSAM, FastSAMPrompt
import torch
import yaml

# from fastsam import FastSAM, FastSAMPrompt
# import torch


class ImageSegmenter(LeafSystem):
    def __init__(self, camera: str):
        """
        Focus coordinates are in the format ((top_left_x, top_left_y), (bottom_right_x, bottom_right_y))
        """

        super().__init__()

        (
            self.top_left_coordinate,
            self.bottom_right_coordinate,
            self.object_coordinates,
            self.background_coordinates,
        ) = self.get_params(camera)

        self.camera = camera

        self.depth_image_input_port = self.DeclareAbstractInputPort(
            f"{camera}_depth_image", AbstractValue.Make(Image[PixelType.kDepth16U]())
        )
        self.color_image_input_port = self.DeclareAbstractInputPort(
            f"{camera}_color_image", AbstractValue.Make(Image[PixelType.kRgba8U]())
        )

        self.masked_depth_image = self.DeclareAbstractOutputPort(
            f"{camera}_masked_depth_image",
            lambda: AbstractValue.Make(Image[PixelType.kDepth16U]()),
            self.MaskDepthImage,
        )

        self.masked_color_image = self.DeclareAbstractOutputPort(
            f"{camera}_masked_color_image",
            lambda: AbstractValue.Make(Image[PixelType.kRgba8U]()),
            self.MaskColorImage,
        )

    def get_params(self, camera):
        file_path = "/home/piplup/piplup/perception/perception_configs.yaml"
        with open(file_path, "r") as file:
            parsed_yaml = yaml.safe_load(file)

            return (
                parsed_yaml[camera]["top_left_coordinate"],
                parsed_yaml[camera]["bottom_right_coordinate"],
                parsed_yaml[camera]["object_points"],
                parsed_yaml[camera]["background_points"],
            )

    def MaskDepthImage(self, context: Context, output: AbstractValue):

        color_image = self.EvalAbstractInput(
            context, self.color_image_input_port.get_index()
        ).get_value()

        depth_image = self.EvalAbstractInput(
            context, self.depth_image_input_port.get_index()
        ).get_value()

        color_image_matrix = np.array(color_image.data, copy=False).reshape(
            color_image.height(), color_image.width(), -1
        )

        mask = self.get_mask(color_image_matrix)

        if mask.shape != (480, 640):
            return
        masked_depth_image = copy.copy(depth_image.data)
        masked_depth_image[~mask] = 0

        img: ImageDepth16U = output.get_mutable_value()
        img.resize(640, 480)
        img.mutable_data[:] = masked_depth_image

    def MaskColorImage(self, context: Context, output: AbstractValue):
        color_image = self.EvalAbstractInput(
            context, self.color_image_input_port.get_index()
        ).get_value()

        color_image_matrix = np.array(color_image.data, copy=False).reshape(
            color_image.height(), color_image.width(), -1
        )

        mask = self.get_mask(color_image_matrix)
        if mask.shape != (480, 640):
            return
        masked_color_image = copy.copy(color_image.data)
        masked_color_image[~mask] = 0

        img: ImageDepth16U = output.get_mutable_value()
        img.resize(640, 480)
        img.mutable_data[:] = masked_color_image

    def get_mask(self, color_image: np.ndarray) -> np.ndarray:

        model = FastSAM("/home/piplup/piplup/perception/weights/FastSAM.pt")
        DEVICE = torch.device(
            "cuda"
            if torch.cuda.is_available()
            else "mps" if torch.backends.mps.is_available() else "cpu"
        )

        everything_results = model(
            cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR),
            device=DEVICE,
            retina_masks=True,
            imgsz=1024,
            conf=0.8,
            iou=0.9,
        )

        prompt_process = FastSAMPrompt(color_image, everything_results, device=DEVICE)

        points = [*self.object_coordinates, *self.background_coordinates]
        pointlabel = [
            *[*[1] * len(self.object_coordinates)],
            *[*[0] * len(self.object_coordinates)],
        ]
        ann = prompt_process.point_prompt(points=points, pointlabel=pointlabel)
        if isinstance(ann, list):
            ann = np.array(ann)
        return ann.squeeze()
