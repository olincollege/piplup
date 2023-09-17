import numpy as np
import os

from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer


meshcat = StartMeshcat()

iiwa7_model_url = (
    "package://drake/manipulation/models/"
    "iiwa_description/iiwa7/iiwa7_with_box_collision.sdf")

# Create a model visualizer and add the robot arm.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModels(url=iiwa7_model_url)

# When this notebook is run in test mode it needs to stop execution without
# user interaction. For interactive model visualization you won't normally
# need the 'loop_once' flag.
test_mode = True if "TEST_SRCDIR" in os.environ else False

# Start the interactive visualizer.
# Click the "Stop Running" button in MeshCat when you're finished.
visualizer.Run(loop_once=test_mode)