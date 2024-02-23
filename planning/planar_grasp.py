import numpy as np
from pydrake.all import *
from copy import copy
from enum import Enum
from common import ConfigureParser
from piplup.utils import AddMeshcatTriad

def GraspCandidateCost(
    meshcat,
    diagram,
    context,
    cloud,
    robotiq_body_index=None,
    plant_system_name="plant",
    scene_graph_system_name="scene_graph",
    adjust_X_G=False,
):
    """
    Args:
        diagram: A diagram containing a MultibodyPlant+SceneGraph that contains
            a free body gripper and any obstacles in the environment that we
            want to check collisions against. It should not include the objects
            in the point cloud; those are handled separately.
        context: The diagram context.  All positions in the context will be
            held fixed *except* the gripper free body pose.
        cloud: a PointCloud in world coordinates which represents candidate
            grasps.
        robotiq_body_index: The body index of the gripper in plant.  If None, then
            a body named "body" will be searched for in the plant.

    Returns:
        cost: The grasp cost

    If adjust_X_G is True, then it also updates the gripper pose in the plant
    context.
    """
    plant = diagram.GetSubsystemByName(plant_system_name)
    plant_context = plant.GetMyMutableContextFromRoot(context)
    scene_graph = diagram.GetSubsystemByName(scene_graph_system_name)
    scene_graph_context = scene_graph.GetMyMutableContextFromRoot(context)
    if robotiq_body_index:
        robotiq = plant.get_body(robotiq_body_index)
    else:
        robotiq = plant.GetBodyByName("robotiq_arg2f_base_link")
        robotiq_body_index = robotiq.index()

    X_G = plant.GetFreeBodyPose(plant_context, robotiq)

    AddMeshcatTriad(
        meshcat, "/gripper_base", length=0.02, radius=0.001, X_PT=X_G
    )

    # Transform cloud into gripper frame
    X_GW = X_G.inverse()
    p_GC = X_GW @ cloud.xyzs()

    # Crop to a region inside of the finger box.
    crop_min = [-0.05, -0.01, 0.11]
    crop_max = [0.05, 0.01, 0.145]
    indices = np.all(
        (
            crop_min[0] <= p_GC[0, :],
            p_GC[0, :] <= crop_max[0],
            crop_min[1] <= p_GC[1, :],
            p_GC[1, :] <= crop_max[1],
            crop_min[2] <= p_GC[2, :],
            p_GC[2, :] <= crop_max[2],
        ),
        axis=0,
    )

    finger_box = Box([0.1, 0.02, 0.035])
    meshcat.SetObject("/finger_box", finger_box, rgba=Rgba(1, 1, 1, 0.5))
    meshcat.SetTransform(
        "/finger_box", X_G @ RigidTransform([0.0, 0.0, 0.1275])
    )

    if adjust_X_G and np.sum(indices) > 0:
        p_GC_x = p_GC[0, indices]
        p_Gcenter_x = (p_GC_x.min() + p_GC_x.max()) / 2.0
        X_G.set_translation(X_G @ np.array([p_Gcenter_x, 0, 0]))
        plant.SetFreeBodyPose(plant_context, robotiq, X_G)
        X_GW = X_G.inverse()

    query_object = scene_graph.get_query_output_port().Eval(scene_graph_context)

    # Check collisions between the gripper and the sink
    if query_object.HasCollisions():
        print("Gripper Collided!")
        cost = np.inf
        return cost

    # Check collisions between the gripper and the point cloud. `margin`` must
    # be smaller than the margin used in the point cloud preprocessing.
    margin = 0.0
    for i in range(cloud.size()):
        distances = query_object.ComputeSignedDistanceToPoint(
            cloud.xyz(i), threshold=margin
        )
        if distances:
            print("Finger Box Collided!")
            cost = np.inf
            return cost

    n_GC = X_GW.rotation().multiply(cloud.normals()[:, indices])

    # Penalize deviation of the gripper from vertical.
    # weight * -dot([0, 0, -1], R_G * [0, 1, 0]) = weight * R_G[2,1]
    cost = 20.0 * X_G.rotation().matrix()[2, 1]

    # Reward sum |dot product of normals with gripper x|^2
    cost -= np.sum(n_GC[0, :] ** 2)

    print(f"Cost: {cost}")
    return cost


def GenerateAntipodalGraspCandidate(
    meshcat,
    diagram,
    context,
    cloud,
    rng,
    robotiq_body_index=None,
    plant_system_name="plant",
    scene_graph_system_name="scene_graph",
):
    """
    Picks a random point in the cloud, and aligns the robot finger with the
    normal of that pixel. The rotation around the normal axis is drawn from a
    uniform distribution over [min_roll, max_roll].

    Args:
        diagram: A diagram containing a MultibodyPlant+SceneGraph that contains
            a free body gripper and any obstacles in the environment that we
            want to check collisions against. It should not include the objects
            in the point cloud; those are handled separately.
        context: The diagram context.  All positions in the context will be
            held fixed *except* the gripper free body pose.
        cloud: a PointCloud in world coordinates which represents candidate
            grasps.
        rng: a np.random.default_rng()
        robotiq_body_index: The body index of the gripper in plant.  If None,
            then a body named "body" will be searched for in the plant.

    Returns:
        cost: The grasp cost X_G: The grasp candidate
    """
    plant = diagram.GetSubsystemByName(plant_system_name)
    plant_context = plant.GetMyMutableContextFromRoot(context)
    scene_graph = diagram.GetSubsystemByName(scene_graph_system_name)
    scene_graph.GetMyMutableContextFromRoot(context)
    if robotiq_body_index:
        robotiq = plant.get_body(robotiq_body_index)
    else:
        robotiq = plant.GetBodyByName("robotiq_arg2f_base_link")
        robotiq_body_index = robotiq.index()

    if cloud.size() < 1:
        return np.inf, None

    index = rng.integers(0, cloud.size() - 1)

    # Use S for sample point/frame.
    p_WS = cloud.xyz(index)
    n_WS = cloud.normal(index)

    # print(f"Chose sample point- p:{p_WS} n:{n_WS}")

    assert np.isclose(
        np.linalg.norm(n_WS), 1.0
    ), f"Normal has magnitude: {np.linalg.norm(n_WS)}"

    Gx = n_WS  # gripper x axis aligns with normal
    # make orthonormal y axis, aligned with world down
    y = np.array([0.0, 0.0, -1.0])
    if np.abs(np.dot(y, Gx)) > 1 - 1e-2:
        # normal was pointing straight down.  reject this sample.
        return np.inf, None

    Gy = np.cross(y, Gx)
    Gz = np.cross(Gx, Gy)

    R_WG = RotationMatrix(np.vstack((Gx, Gy, Gz)).T)

    AddMeshcatTriad(
        meshcat, "/pick_point", length=0.02, radius=0.001, X_PT=RigidTransform(R_WG.ToQuaternion(), p_WS)
    )

    p_GS_G = [0.05 - 0.01, 0.0, 0.1275]

    # Try orientations from the center out
    min_roll = -np.pi / 3.0
    max_roll = np.pi / 3.0
    alpha = np.array([0.5, 0.65, 0.35, 0.8, 0.2, 1.0, 0.0])

    for theta in min_roll + (max_roll - min_roll) * alpha:
        # Rotate the object in the hand by a random rotation (around the
        # normal).
        R_WG2 = R_WG.multiply(RotationMatrix.MakeXRotation(theta))

        # Use G for gripper frame.
        p_SG_W = -R_WG2.multiply(p_GS_G)
        p_WG = p_WS + p_SG_W

        X_G = RigidTransform(R_WG2, p_WG)

        plant.SetFreeBodyPose(plant_context, robotiq, X_G)
        cost = GraspCandidateCost(meshcat, diagram, context, cloud, adjust_X_G=True)
        X_G = plant.GetFreeBodyPose(plant_context, robotiq)
        if np.isfinite(cost):
            print(f"Cost: {cost}")
            return cost, X_G

    return np.inf, None

def make_internal_model(meshcat):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    ConfigureParser(parser)
    parser.AddModelsFromUrl(
        "package://piplup_models/robotiq_description/sdf/robotiq_2f_85_static.sdf"
    )
    parser.AddModelsFromUrl(
        "package://piplup_models/scope_station/models/scope_table.sdf"
    )
    plant.Finalize()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    # vis = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    return builder.Build()


class PlanarGraspSelector(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        self.DeclareAbstractInputPort("merged_point_cloud", AbstractValue.Make(PointCloud(0)))
        port = self.DeclareAbstractOutputPort(
            "grasp_selection",
            lambda: AbstractValue.Make((np.inf, np.inf, RigidTransform())),
            self.SelectGrasp,
        )

        self.meshcat: Meshcat = Meshcat()

        self._internal_model = make_internal_model(self.meshcat)
        self._internal_model_context = self._internal_model.CreateDefaultContext()
        self._rng = np.random.default_rng()

    def SelectGrasp(self, context, output):
        pcd = self.get_input_port(0).Eval(context)

        costs = []
        X_Gs = []

        for i in range(1):
            cost, X_G = GenerateAntipodalGraspCandidate(
                self.meshcat,
                self._internal_model,
                self._internal_model_context,
                pcd,
                self._rng,
            )
            if np.isfinite(cost):
                costs.append(cost)
                X_Gs.append(X_G)

        if len(costs) == 0:
            # No viable grasp candidates found
            X_WG = RigidTransform(
                RollPitchYaw(0, np.pi, 0), [0.5, 0, 0.4]
            )
            output.set_value((np.inf, np.inf, X_WG))
        else:
            best = np.argmin(costs)
            graspability = np.average(costs)

            print(f"Graspability: {graspability}")
            output.set_value((graspability, costs[best], X_Gs[best]))
        
        self._internal_model.ForcedPublish(self._internal_model_context)