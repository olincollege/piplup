import numpy as np
from dataclasses import dataclass
from pydrake.all import *

@dataclass
class Mass:
    mass: float

@dataclass
class Shape:
    pcd: PointCloud

@dataclass
class Semantics:
    identity: str

@dataclass
class Rigidity:
    compliance: float

@dataclass
class Porosity:
    pressure: float

@dataclass
class StaticCOM:
    com: np.array
