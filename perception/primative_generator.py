"""
System for fitting primatives to point clouds
"""

import numpy as np
import matplotlib.pyplot as plt
from pydrake.all import *
from pydrake.geometry import (
    Meshcat,
)


class PrimitiveGenerator(LeafSystem):
    def __init__(self, meshcat: Meshcat = None):
        super().__init__()
        