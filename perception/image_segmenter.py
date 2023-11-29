import cv2
import numpy as np
from pydrake.all import *
import builtins
import matplotlib.pyplot as plt

class ImageSegmenter(LeafSystem):
    def __init__(self, camera: str, hz: int = 60):
        super().__init__()
        self.camera = camera

        self.depth_image_input_port = self.DeclareAbstractInputPort("depth_image", AbstractValue.Make(Image[PixelType.kDepth32F]()))
        self.color_image_input_port = self.DeclareAbstractInputPort("color_image",   AbstractValue.Make(Image[PixelType.kRgba8U]()))

        self.masked_depth_image = self.DeclareAbstractOutputPort(f"{camera}_masked_depth_image", lambda: AbstractValue.Make(Image[PixelType.kDepth32F]()), self.MaskDepthImage)

    def MaskDepthImage(self, context: Context, output: AbstractValue):
        color_image = self.EvalAbstractInput(context, self.color_image_input_port.get_index()).get_value()
        color_image_matrix = np.array(color_image.data, copy=False).reshape(color_image.height(), color_image.width(), -1)

        x, y, w, h = self.get_mask(color_image_matrix)
        depth_image = self.EvalAbstractInput(context, self.depth_image_input_port.get_index()).get_value()
        # Apply the mask to the depth image
        if x and y and w and h:
            # Iterate over the entire image
            for i in range(depth_image.height()):
                for j in range(depth_image.width()):
                    # Check if the current pixel is outside the specified box
                    if i < y or i >= y + h or j < x or j >= x + w:
                        depth_image.at(j, i)[0] = 0  # Set the value to zero outside the box

        output.set_value(depth_image)

    def get_mask(self, color_image: np.ndarray) -> np.ndarray:
        #TODO: Only works with Cheez It box - doesn't create bounding box around entire thing. Replace with better segmentation.
        image_hsv = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)

        # Define the range of red color in HSV
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Threshold the HSV image to get only red colors
        mask1 = cv2.inRange(image_hsv, lower_red, upper_red)
        mask2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours from the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Assuming we want to create a bounding box around the largest red object
        if contours:
            # Find the largest contour based on area
            largest_contour = builtins.max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            return x, y, w, h
        return None, None, None, None