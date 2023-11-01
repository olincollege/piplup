from pydrake.all import *
from pydrake.geometry import SceneGraph
import numpy as np
from suction_gripper import (
    CupPressureSource,
    CupObjInterface,
    ExampleGripperMultibodyModel,
)


builder = DiagramBuilder()
kMultibodyTimeStep = 0.002
plant = MultibodyPlant(kMultibodyTimeStep)
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, plant)

kPackageMass = 2.73
kPackageLen = 0.3
kPackageWidth = 0.15
obj_body = plant.AddRigidBody(
    "obj_body",
    SpatialInertia.SolidBoxWithMass(
        kPackageMass, kPackageLen, kPackageWidth, kPackageLen
    ),
)

plant.RegisterVisualGeometry(
    obj_body,
    RigidTransform(),
    Box(kPackageLen, kPackageWidth, kPackageLen),
    "obj_body_visual_geom",
    np.array([0.6, 0.4, 0.1, 1.0]),
)

kWristMass = 1.0
kWristInertia = 0.01
kWristHeight = 1.0
wrist_body = plant.AddRigidBody(
    "wrist_body",
    SpatialInertia(
        kWristMass,
        np.zeros(3),
        UnitInertia(kWristInertia, kWristInertia, kWristInertia),
    ),
)

plant.WeldFrames(
    plant.world_frame(),
    wrist_body.body_frame(),
    RigidTransform(np.array([0.0, 0.0, kWristHeight])),
)

obj_body_collision_geom = plant.RegisterCollisionGeometry(
    obj_body,
    RigidTransform(),
    Box(kPackageLen, kPackageWidth, kPackageLen),
    "obj_body_collision_geom",
    CoulombFriction(1.0,1.0),
)

suction_gripper = ExampleGripperMultibodyModel(plant, wrist_body)


plant.set_discrete_contact_solver(DiscreteContactSolver.kSap)
plant.Finalize()



suction_cup_act_pt_geom_id_vec = suction_gripper.get_suction_cup_act_pt_geom_id_vec()
suction_cup_act_pt_geom_id_to_body_idx_map = (
    suction_gripper.get_suction_cup_act_pt_geom_id_to_body_idx_map()
)
suction_cup_edge_pt_geom_id_vec = suction_gripper.get_suction_cup_edge_pt_geom_id_vec()
kPumpPressure = -9e4
kMaxSuctionDist = 0.004
kNumSuctionCup = 1
kSuctionModelTimeStep = 0.01

suction_pressure_source = builder.AddSystem(
    CupPressureSource(kPumpPressure, kMaxSuctionDist, kNumSuctionCup)
)
obj_geom_id_to_body_idx_map = dict()
obj_geom_id_to_body_idx_map[obj_body_collision_geom] = obj_body.index()
cup_obj_interface = builder.AddSystem(
    CupObjInterface(
        kSuctionModelTimeStep,
        suction_gripper.CalcCupArea(),
        suction_cup_act_pt_geom_id_vec,
        suction_cup_act_pt_geom_id_to_body_idx_map,
        suction_cup_edge_pt_geom_id_vec,
        obj_geom_id_to_body_idx_map,
    )
)

builder.Connect(
    suction_pressure_source.GetSuctionCupPressureOutputPort(),
    cup_obj_interface.GetSuctionCupPressureInputPort(),
)
builder.Connect(
    cup_obj_interface.GetCupObjDistOutputPort(),
    suction_pressure_source.GetCupObjDistInputPort(),
)
builder.Connect(
    scene_graph.get_query_output_port(), cup_obj_interface.GetGeomQueryInputPort()
)
builder.Connect(
    cup_obj_interface.GetSuctionForceOutputPort(),
    plant.get_applied_spatial_force_input_port(),
)


AddDefaultVisualization(builder)
diagram = builder.Build()

simulator = Simulator(diagram)

plant_context = plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())

kCupCenterHeightFromGround = 0.623
kCupEngageOffset = 0.01
plant.SetFreeBodyPose(
    plant_context,
    obj_body,
    RigidTransform(
        RollPitchYaw(0, np.pi / 4,0).ToQuaternion(),
        np.array(
            [
                -kPackageLen / 2 * sin(np.pi / 4),
                0,
                kCupCenterHeightFromGround
                + kCupEngageOffset
                - kPackageLen / 2 * cos(np.pi / 4),
            ]
        ),
    ),
)
#   drake::math::RollPitchYaw<double>(0., M_PI/4, 0.),
#   Eigen::Vector3d(-kPackageLen / 2 * sin(M_PI/4), 0,
#                   kCupCenterHeightFromGround + kCupEngageOffset -
#                       kPackageLen / 2 * cos(M_PI/4))));
suction_pressure_source_context = suction_pressure_source.GetMyMutableContextFromRoot(
    simulator.get_mutable_context()
)
suction_pressure_source.GetSuctionCmdInputPort().FixValue(
    suction_pressure_source_context, np.ones(1)
)

simulator.set_target_realtime_rate(1)
simulator.Initialize()

simulator.AdvanceTo(150.0)
