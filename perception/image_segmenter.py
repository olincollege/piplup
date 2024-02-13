import cv2
import numpy as np
from pydrake.all import *
import builtins
import matplotlib.pyplot as plt
from fastsam import FastSAM, FastSAMPrompt
import torch


class ImageSegmenter(LeafSystem):
    def __init__(self, camera: str, top_left_coordinate: tuple[int, int] = None, bottom_right_coordinate: tuple[int, int] = None, object_coordinates: list[tuple[int, int]] = None, 
                 background_coordinates: list[tuple[int, int]] = None, hz: int = 60):
        '''
            Focus coordinates are in the format ((top_left_x, top_left_y), (bottom_right_x, bottom_right_y))
        '''

        super().__init__()

        self.top_left_coordinate = top_left_coordinate
        self.bottom_right_coordinate = bottom_right_coordinate
        self.background_coordinates = background_coordinates
        self.object_coordinates = object_coordinates
        self.background_coordinates = background_coordinates
        
        self.camera = camera

        self.depth_image_input_port = self.DeclareAbstractInputPort(
            f"{camera}_depth_image", AbstractValue.Make(Image[PixelType.kDepth32F]())
        )
        self.color_image_input_port = self.DeclareAbstractInputPort(
            f"{camera}_color_image", AbstractValue.Make(Image[PixelType.kRgba8U]())
        )

        self.masked_depth_image = self.DeclareAbstractOutputPort(
            f"{camera}_masked_depth_image",
            lambda: AbstractValue.Make(Image[PixelType.kDepth32F]()),
            self.MaskDepthImage,
        )

    def MaskDepthImage(self, context: Context, output: AbstractValue):
        color_image = self.EvalAbstractInput(
            context, self.color_image_input_port.get_index()
        ).get_value()

        if self.top_left_coordinate and self.bottom_right_coordinate:
            top_left_x, top_left_y = self.top_left_coordinate
            bottom_right_x, bottom_right_y = self.bottom_right_coordinate
            print(top_left_x, top_left_y, bottom_right_x, bottom_right_y)
            print(color_image.data.shape)
            mask = np.zeros((480, 640))

            mask[top_left_y:bottom_right_y+1, top_left_x:bottom_right_x+1] = 1

            for i in range(color_image.height()):
                for j in range(color_image.width()):
                    if mask[i][j] == 0:
                        for c in range(4):
                            color_image.at(j, i)[c] = 0
            
        color_image_matrix = np.array(color_image.data, copy=False).reshape(
            color_image.height(), color_image.width(), -1
        )

        mask = self.get_mask(color_image_matrix)

        depth_image = self.EvalAbstractInput(
            context, self.depth_image_input_port.get_index()
        ).get_value()

        for i in range(depth_image.height()):
            for j in range(depth_image.width()):
                if mask[i][j] == 0:
                    depth_image.at(j, i)[0] = 0

        output.set_value(depth_image)

    def get_mask(self, color_image: np.ndarray) -> np.ndarray:

        model = FastSAM('/home/piplup/piplup/perception/weights/FastSAM.pt')
        DEVICE = torch.device(
            "cuda"
            if torch.cuda.is_available()
            else "mps"
            if torch.backends.mps.is_available()
            else "cpu"
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

        if self.object_coordinates and self.background_coordinates:
            points = [*self.object_coordinates, *self.background_coordinates]
            pointlabel = [*[*[1]*len(self.object_coordinates)], *[*[0]*len(self.object_coordinates)]]
        else:
            points = [[0, 0], [1, 1]]
            pointlabel = [0, 1]

        ann = prompt_process.point_prompt(points=points, pointlabel=pointlabel)
        return ann.squeeze()
