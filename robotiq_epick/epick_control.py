from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
import numpy as np
from suction_gripper import CupPressureSource, CupObjInterface

# Build Controls
def BuildEPickControl():
    pass


def AddSimEPickDriver(
    sim_plant: MultibodyPlant,
    suction_gripper,
    obj_body : Body,
    builder: DiagramBuilder,
) -> System:
    inner_name = f"EPickDriver(epick_2cup)"
    inner_builder = DiagramBuilder()

    suction_cup_act_pt_geom_id_vec = suction_gripper.get_suction_cup_act_pt_geom_id_vec()
    suction_cup_act_pt_geom_id_to_body_idx_map = (
        suction_gripper.get_suction_cup_act_pt_geom_id_to_body_idx_map()
    )
    suction_cup_edge_pt_geom_id_vec = suction_gripper.get_suction_cup_edge_pt_geom_id_vec()
    kPumpPressure = -9e4
    kMaxSuctionDist = 0.004
    kNumSuctionCup = 1
    kSuctionModelTimeStep = 0.01

    suction_pressure_source = inner_builder.AddSystem(
        CupPressureSource(kPumpPressure, kMaxSuctionDist, kNumSuctionCup)
    )
    obj_geom_id_to_body_idx_map = dict()
    for obj_body_collision_geom in sim_plant.GetCollisionGeometriesForBody(obj_body):
        obj_geom_id_to_body_idx_map[obj_body_collision_geom] = obj_body.index()
    cup_obj_interface = inner_builder.AddSystem(
        CupObjInterface(
            kSuctionModelTimeStep,
            suction_gripper.CalcCupArea(),
            suction_cup_act_pt_geom_id_vec,
            suction_cup_act_pt_geom_id_to_body_idx_map,
            suction_cup_edge_pt_geom_id_vec,
            obj_geom_id_to_body_idx_map,
        )
    )

    inner_builder.Connect(
        suction_pressure_source.GetSuctionCupPressureOutputPort(),
        cup_obj_interface.GetSuctionCupPressureInputPort(),
    )
    inner_builder.Connect(
        cup_obj_interface.GetCupObjDistOutputPort(),
        suction_pressure_source.GetCupObjDistInputPort(),
    )

    inner_builder.ExportInput(cup_obj_interface.GetGeomQueryInputPort(), "geometry_query") 
    inner_builder.ExportOutput(cup_obj_interface.GetSuctionForceOutputPort(), "suction_force")
    inner_builder.ExportInput(suction_pressure_source.GetSuctionCmdInputPort(), "suction_command")
    diagram = inner_builder.Build()
    builder.AddNamedSystem(inner_name, diagram)
    return diagram