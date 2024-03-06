import numpy as np
from pydrake.all import *
from copy import copy
from enum import Enum
from common import ConfigureParser
from common.utils import *
from common.logging import *
import time


def GraspCandidateCost(
    meshcat: Meshcat,
    diagram: Diagram,
    context: Context,
    cloud: PointCloud,
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
    plant: MultibodyPlant = diagram.GetSubsystemByName(plant_system_name)
    plant_context: Context = plant.GetMyMutableContextFromRoot(context)
    scene_graph: SceneGraph = diagram.GetSubsystemByName(scene_graph_system_name)
    scene_graph_context: Context = scene_graph.GetMyMutableContextFromRoot(context)
    if robotiq_body_index:
        robotiq = plant.get_body(robotiq_body_index)
    else:
        robotiq = plant.GetBodyByName("robotiq_arg2f_base_link")
        robotiq_body_index = robotiq.index()

    X_G = plant.GetFreeBodyPose(plant_context, robotiq)

    # Transform cloud into gripper frame
    X_GW = X_G.inverse()
    p_GC = X_GW @ cloud.xyzs()

    # Crop to a region inside of the finger box.
    crop_min = [-0.01, -0.0425, 0.115]
    crop_max = [0.01, 0.0425, 0.15]
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

    finger_box = Box([0.02, 0.085, 0.035])
    meshcat.SetObject("/finger_box", finger_box, rgba=Rgba(0, 0, 1, 0.5))
    meshcat.SetTransform("/finger_box", X_G @ RigidTransform([0.0, 0.0, 0.1325]))

    grip_width = np.inf

    if np.sum(indices) > 0:
        p_GC_y = p_GC[1, indices]
        grip_width = p_GC_y.max() - p_GC_y.min()

    if adjust_X_G and np.sum(indices) > 0:
        p_GC_y = p_GC[1, indices]
        grip_width = p_GC_y.max() - p_GC_y.min()
        p_Gcenter_y = (p_GC_y.min() + p_GC_y.max()) / 2.0
        X_G.set_translation(X_G @ np.array([0, p_Gcenter_y, 0]))
        plant.SetFreeBodyPose(plant_context, robotiq, X_G)
        X_GW = X_G.inverse()

    AddMeshcatTriad(meshcat, "/gripper_base", length=0.02, radius=0.001, X_PT=X_G)

    query_object: QueryObject = scene_graph.get_query_output_port().Eval(
        scene_graph_context
    )

    # Check collisions between the gripper and the sink
    if query_object.HasCollisions():
        logging.debug("Gripper collided with World!")
        cost = np.inf
        return cost, np.inf

    # Check collisions between the gripper and the point cloud. `margin`` must
    # be smaller than the margin used in the point cloud preprocessing.
    margin = 0.0

    for i in range(cloud.size()):
        distances = query_object.ComputeSignedDistanceToPoint(
            cloud.xyz(i), threshold=margin
        )
        geometry_names = [query_object.inspector().GetName(d.id_G) for d in distances]
        keep_idx = ["table" not in n and "extrusion" not in n for n in geometry_names]
        collisions = np.array(distances)[keep_idx]

        if len(collisions) > 0:
            # meshcat.SetObject("/collision_point", Sphere(0.005), rgba=Rgba(1, 0, 0, 1))
            # meshcat.SetTransform("/collision_point", RigidTransform(cloud.xyz(i)))
            # time.sleep(3)
            logging.debug("Gripper collided with point cloud!")
            cost = np.inf
            return cost, np.inf

    n_GC = X_GW.rotation().multiply(cloud.normals()[:, indices])

    # Penalize deviation of the gripper from vertical.
    # weight * -dot([0, 0, -1], R_G * [0, 1, 0]) = weight * R_G[2,1]
    cost = 20.0 * np.abs(X_G.rotation().matrix()[2, 0])

    # Reward sum |dot product of normals with gripper y|^2
    cost -= np.sum(n_GC[1, :] ** 2)

    return cost, grip_width


def GenerateAntipodalGraspCandidate(
    meshcat: Meshcat,
    diagram: Diagram,
    context: Context,
    cloud: PointCloud,
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
    plant: MultibodyPlant = diagram.GetSubsystemByName(plant_system_name)
    plant_context: Context = plant.GetMyMutableContextFromRoot(context)
    scene_graph: SceneGraph = diagram.GetSubsystemByName(scene_graph_system_name)
    scene_graph.GetMyMutableContextFromRoot(context)
    if robotiq_body_index:
        robotiq = plant.get_body(robotiq_body_index)
    else:
        robotiq = plant.GetBodyByName("robotiq_arg2f_base_link")
        robotiq_body_index = robotiq.index()

    if cloud.size() < 1:
        return np.inf, None, np.inf

    index = rng.integers(0, cloud.size() - 1)

    # Use S for sample point/frame.
    p_WS = cloud.xyz(index)
    n_WS = cloud.normal(index)

    # logging.debug(f"Chose sample point- p:{p_WS} n:{n_WS}")

    assert np.isclose(
        np.linalg.norm(n_WS), 1.0
    ), f"Normal has magnitude: {np.linalg.norm(n_WS)}"

    Gy = n_WS  # gripper y axis aligns with normal
    # make orthonormal z axis, aligned with world down
    z = np.array([0.0, 0.0, -1.0])
    if np.abs(np.dot(z, Gy)) > 1 - 1e-2:
        # normal was pointing straight down.  reject this sample.
        return np.inf, None, np.inf

    Gx = np.cross(Gy, z)
    Gz = np.cross(Gx, Gy)

    R_WG = RotationMatrix(np.vstack((Gx, Gy, Gz)).T)

    p_GS_G = [0.0, 0.0425 - 0.005, 0.1325]

    AddMeshcatTriad(
        meshcat,
        "/pick_point",
        length=0.02,
        radius=0.001,
        X_PT=RigidTransform(R_WG.ToQuaternion(), p_WS),
    )

    # Try orientations from the center out
    min_roll = -np.pi / 2.0
    max_roll = np.pi / 2.0
    alpha = np.array([0.5, 0.65, 0.35, 0.8, 0.2, 1.0, 0.0])

    for theta in min_roll + (max_roll - min_roll) * alpha:
        # Rotate the object in the hand by a random rotation (around the
        # normal).
        R_WG2 = R_WG.multiply(RotationMatrix.MakeYRotation(theta))

        # Use G for gripper frame.
        p_SG_W = -R_WG2.multiply(p_GS_G)
        p_WG = p_WS + p_SG_W

        X_G = RigidTransform(R_WG2, p_WG)

        plant.SetFreeBodyPose(plant_context, robotiq, X_G)
        cost, grip_width = GraspCandidateCost(
            meshcat, diagram, context, cloud, adjust_X_G=True
        )
        X_G = plant.GetFreeBodyPose(plant_context, robotiq)

        # diagram.ForcedPublish(context)

        if np.isfinite(cost):
            # logging.debug(f"Cost: {cost}")
            return cost, X_G, grip_width

    return np.inf, None, np.inf


def make_internal_model(meshcat):
    builder = DiagramBuilder()
    plant: MultibodyPlant
    scene_graph: SceneGraph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    ConfigureParser(parser)
    parser.AddModelsFromUrl(
        "package://piplup_models/robotiq_description/sdf/robotiq_2f_85_static_primitive_collision.sdf"
    )
    parser.AddModelsFromUrl(
        "package://piplup_models/scope_station/models/scope_table.sdf"
    )

    model_inspector: SceneGraphInspector = scene_graph.model_inspector()
    geometry_ids = model_inspector.GetAllGeometryIds()
    geometry_names = [model_inspector.GetName(id) for id in geometry_ids]
    valid_collision_idx = np.array(
        ["table" in n or "extrusion" in n for n in geometry_names]
    )
    excluded_bodies = np.array(geometry_ids)[~valid_collision_idx]
    gripper_exclude = CollisionFilterDeclaration().ExcludeWithin(
        GeometrySet(excluded_bodies)
    )
    scene_graph.collision_filter_manager().Apply(gripper_exclude)

    plant.Finalize()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    # vis = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    return builder.Build()


class PlanarGraspSelector(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        self.DeclareAbstractInputPort(
            "merged_point_cloud", AbstractValue.Make(PointCloud(0))
        )
        port = self.DeclareAbstractOutputPort(
            "grasp_selection",
            lambda: AbstractValue.Make((np.inf, np.inf, RigidTransform(), np.inf)),
            self.SelectGrasp,
        )

        self.meshcat: Meshcat = Meshcat()

        self._internal_model: Diagram = make_internal_model(self.meshcat)
        self._internal_model_context: Context = (
            self._internal_model.CreateDefaultContext()
        )
        self._rng = np.random.default_rng()

    def SelectGrasp(self, context, output):
        pcd = self.get_input_port(0).Eval(context)

        self.meshcat.SetObject(
            "/point_cloud", pcd, point_size=0.003, rgba=Rgba(1, 1, 1, 0.5)
        )

        costs = []
        X_Gs = []
        grip_widths = []
        grasp_attempts = 100

        for _ in range(grasp_attempts):
            cost, X_G, grip_width = GenerateAntipodalGraspCandidate(
                self.meshcat,
                self._internal_model,
                self._internal_model_context,
                pcd,
                self._rng,
            )
            if np.isfinite(cost):
                costs.append(cost)
                X_Gs.append(X_G)
                grip_widths.append(grip_width)

        if len(costs) == 0:
            # No viable grasp candidates found
            X_WG = RigidTransform(RollPitchYaw(0, np.pi, 0), [0.5, 0, 0.4])
            output.set_value((np.inf, np.inf, X_WG, np.inf))
        else:
            best = np.argmin(costs)
            # Percentage of valid grasps found
            graspability = len(costs) / grasp_attempts

            logging.info(f"Best Grasp Score: {costs[best]}")
            logging.info(f"Graspability: {graspability}")
            logging.info(f"Grasp width: {grip_widths[best]}")

            plant: MultibodyPlant = self._internal_model.GetSubsystemByName("plant")
            plant_context = plant.GetMyMutableContextFromRoot(
                self._internal_model_context
            )
            robotiq = plant.GetBodyByName("robotiq_arg2f_base_link")

            finger_box = Box([0.02, 0.085, 0.035])
            self.meshcat.SetObject("/finger_box", finger_box, rgba=Rgba(0, 1, 0, 0.5))
            self.meshcat.SetTransform(
                "/finger_box", X_Gs[best] @ RigidTransform([0.0, 0.0, 0.1325])
            )
            plant.SetFreeBodyPose(plant_context, robotiq, X_Gs[best])
            self._internal_model.ForcedPublish(self._internal_model_context)
            output.set_value((graspability, costs[best], X_Gs[best], grip_widths[best]))
