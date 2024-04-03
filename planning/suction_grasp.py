import numpy as np
from pydrake.all import *
from pydrake.geometry import Meshcat, StartMeshcat

from common.utils import AddMeshcatTriad


class SuctionGraspSelector(LeafSystem):
    def __init__(self, meshcat):
        LeafSystem.__init__(self)

        self.point_cloud_port_ = self.DeclareAbstractInputPort(
            "merged_point_cloud", AbstractValue.Make(PointCloud())
        )
        self.meshcat: Meshcat = meshcat
        self.DeclareAbstractOutputPort(
            "grasp_selection",
            lambda: AbstractValue.Make((np.inf, RigidTransform())),
            self.GenerateGrasp,
        )

        self._rng = np.random.default_rng()
        self.cup_diam = 0.04
        self.cup_depth = 0.008
        self.meshcat.SetObject(
            "/cropping_box",
            Cylinder(self.cup_diam / 2, self.cup_depth),
            rgba=Rgba(0, 1, 0, 0.25),
        )

        self.X_end_to_tool_inv = RigidTransform(
            AngleAxis(np.pi, np.array([1, 0, 0])), np.array([0, 0, 0.06])
        )

    def GenerateGrasp(self, context: Context, output: AbstractValue):
        pc: PointCloud = self.point_cloud_port_.Eval(context)

        centroid = np.mean(pc.xyzs(), axis=1)
        AddMeshcatTriad(
            self.meshcat,
            "/centroid",
            length=0.05,
            radius=0.003,
            X_PT=RigidTransform(centroid),
        )

        candidates = []
        scores = []
        crops = []
        for i in range(1000):
            index = self._rng.integers(0, pc.size() - 1)
            p_WS = pc.xyz(index)
            n_WS = pc.normal(index)
            if np.abs(np.dot(np.array([0.0, 0.0, -1.0]), n_WS)) < 1e-6:
                continue

            # Crop to surrounding region
            Gy = np.cross(n_WS, np.array([0, 0, 1]))
            Gy = Gy / np.linalg.norm(Gy)  # normalize
            Gx = np.cross(Gy, n_WS)
            R_WG = RotationMatrix(np.vstack((Gx, Gy, n_WS)).T)
            lower = R_WG @ (
                -np.array([self.cup_diam, self.cup_diam, self.cup_depth]) / 2
            )
            upper = R_WG @ (
                np.array([self.cup_diam, self.cup_diam, self.cup_depth]) / 2
            )
            p1 = np.minimum(lower, upper)
            p2 = np.maximum(lower, upper)
            if np.all(p1 > p2) or np.any(np.isnan(p1)) or np.any(np.isnan(p2)):
                continue
            cropped = pc.Crop(p_WS + p1, p_WS + p2)

            if (
                cropped.size() > 70
                and np.abs(np.dot(np.array([0.0, 0.0, 1.0]), n_WS)) > 0.8
                and p_WS[2] > 0.02
            ):
                score = 5 * p_WS[2] + 1 / np.linalg.norm(p_WS[:2] - centroid[:2])
                scores.append(score)
                candidates.append(RigidTransform(R_WG, p_WS))
                crops.append(cropped)
            self.meshcat.SetObject(
                "/cropping_box",
                Cylinder(self.cup_diam / 2, self.cup_depth),
                rgba=Rgba(0, 0, 1, 0.25),
            )
            self.meshcat.SetTransform("/cropping_box", RigidTransform(R_WG, p_WS))

        if scores:
            i = np.array(scores).argmax()
            G = candidates[i]
            cropped = crops[i]

            self.meshcat.SetObject(
                "/cropped", cropped, point_size=0.003, rgba=Rgba(0, 1, 0, 1)
            )
            self.meshcat.SetTransform("/cropping_box", G)
            AddMeshcatTriad(
                self.meshcat, "/pick_point", length=0.02, radius=0.001, X_PT=G
            )
            AddMeshcatTriad(
                self.meshcat,
                "/ee_pose",
                length=0.02,
                radius=0.001,
                X_PT=G @ self.X_end_to_tool_inv,
            )
            # X_WT = X_WE * X_ET
            output.set_value((scores[i], G @ self.X_end_to_tool_inv))
        else:
            output.set_value((0, RigidTransform()))
