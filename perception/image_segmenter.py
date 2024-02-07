import cv2
import numpy as np
from pydrake.all import *
import builtins
import matplotlib.pyplot as plt
from fastsam import FastSAM, FastSAMPrompt
import torch


class ImageSegmenter(LeafSystem):
    def __init__(self, camera: str, focus_coordinates: list[tuple[int, int]] = None, object_coordinates: list[tuple[int, int]] = None, 
                 background_coordinates: list[tuple[int, int]] = None, hz: int = 60):
        '''
            Focus coordinates are in the format ((top_left_x, top_left_y), (bottom_right_x, bottom_right_y))
        '''

        super().__init__()

        self.focus_coordinates = focus_coordinates
        self.object_coordinates = object_coordinates
        self.background_coordinates = background_coordinates
        
        self.camera = camera

        self.depth_image_input_port = self.DeclareAbstractInputPort(
            "depth_image", AbstractValue.Make(Image[PixelType.kDepth32F]())
        )
        self.color_image_input_port = self.DeclareAbstractInputPort(
            "color_image", AbstractValue.Make(Image[PixelType.kRgba8U]())
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
        color_image_matrix = np.array(color_image.data, copy=False).reshape(
            color_image.height(), color_image.width(), -1
        )

        mask = self.get_mask(color_image_matrix)
        print(mask.shape)
        depth_image = self.EvalAbstractInput(
            context, self.depth_image_input_port.get_index()
        ).get_value()
        print(depth_image.shape)
        for i in range(depth_image.height()):
            for j in range(depth_image.width()):
                if mask[i][j] == 0:
                    depth_image.at(j, i)[0] = 0

        output.set_value(depth_image)

    def get_mask(self, color_image: np.ndarray, bounding_box: List[int] = None) -> np.ndarray:

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
            conf=0.4,
            iou=0.9,
        )

        prompt_process = FastSAMPrompt(color_image, everything_results, device=DEVICE)
        ann = prompt_process.point_prompt(points=[[220, 350],[300,190]], pointlabel=[1, 0])
        return ann.squeeze()
