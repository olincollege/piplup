import numpy as np
from pydrake.all import *
from pydrake.geometry import Meshcat


# Credit to https://github.com/RussTedrake/manipulation
def AddMeshcatTriad(
    meshcat: Meshcat,
    path: str,
    length: float = 0.25,
    radius: float = 0.01,
    opacity: float = 1.0,
    X_PT: RigidTransform = RigidTransform(),
):
    """Adds an X-Y-Z triad to the meshcat scene.

    Args:
        meshcat: A Meshcat instance.
        path: The Meshcat path on which to attach the triad. Using relative paths will attach the triad to the path's coordinate system.
        length: The length of the axes in meters.
        radius: The radius of the axes in meters.
        opacity: The opacity of the axes in [0, 1].
        X_PT: The pose of the triad relative to the path.
    """
    meshcat.SetTransform(path, X_PT)
    # x-axis
    X_TG = RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2), [length / 2.0, 0, 0])
    meshcat.SetTransform(path + "/x-axis", X_TG)
    meshcat.SetObject(
        path + "/x-axis", Cylinder(radius, length), Rgba(1, 0, 0, opacity)
    )

    # y-axis
    X_TG = RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2), [0, length / 2.0, 0])
    meshcat.SetTransform(path + "/y-axis", X_TG)
    meshcat.SetObject(
        path + "/y-axis", Cylinder(radius, length), Rgba(0, 1, 0, opacity)
    )

    # z-axis
    X_TG = RigidTransform([0, 0, length / 2.0])
    meshcat.SetTransform(path + "/z-axis", X_TG)
    meshcat.SetObject(
        path + "/z-axis", Cylinder(radius, length), Rgba(0, 0, 1, opacity)
    )