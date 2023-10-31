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
plant: MultibodyPlant
scene_graph: SceneGraph
plant, scene_graph = AddMultibodyPlantSceneGraph(
    builder, MultibodyPlant(kMultibodyTimeStep)
)
world_body = plant.world_body()

kFrictionCoeff = 1.0
obj_friction = CoulombFriction(kFrictionCoeff, kFrictionCoeff)

kPackageMass = 2.73
kPackageLen = 0.3
kPackageWidth = 0.15
obj_body = plant.AddRigidBody(
    "obj_body",
    SpatialInertia.SolidBoxWithMass(
        kPackageMass, kPackageLen, kPackageWidth, kPackageLen
    ),
)

obj_body_collision_geom = plant.RegisterCollisionGeometry(
    obj_body,
    RigidTransform(),
    Box(kPackageLen, kPackageWidth, kPackageLen),
    "obj_body_collision_geom",
    obj_friction,
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

plant.AddJoint(
    WeldJoint(
        "wrist_joint",
        world_body,
        RigidTransform(),
        wrist_body,
        RigidTransform(),
        RigidTransform(np.array([0.0, 0.0, kWristHeight])),
    )
)

suction_gripper = ExampleGripperMultibodyModel(plant, wrist_body)
suction_cup_act_pt_geom_id_vec = suction_gripper.get_suction_cup_act_pt_geom_id_vec()
suction_cup_act_pt_geom_id_to_body_idx_map = (
    suction_gripper.get_suction_cup_act_pt_geom_id_to_body_idx_map()
)
suction_cup_edge_pt_geom_id_vec = suction_gripper.get_suction_cup_edge_pt_geom_id_vec()

plant.set_discrete_contact_solver(DiscreteContactSolver.kSap)
plant.Finalize()

kPumpPressure = -9e4
kMaxSuctionDist = 0.004
kNumSuctionCup = 1
kSuctionModelTimeStep = 0.01

suction_pressure_source = builder.AddSystem(
    CupPressureSource(kPumpPressure, kMaxSuctionDist, kNumSuctionCup)
)
obj_geom_id_to_body_idx_map = {obj_body_collision_geom, obj_body.index()}
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
diagram = builder.Build()

simulator = Simulator(diagram)

plant_context = plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())

suction_pressure_source_context = suction_pressure_source.GetMyMutableContextFromRoot(
    simulator.get_mutable_context()
)
suction_pressure_source.GetSuctionCmdInputPort().FixValue(
    suction_pressure_source_context, np.ones(1)
)

simulator.set_target_realtime_rate(1)
simulator.Initialize()

simulator.AdvanceTo(150.0)
