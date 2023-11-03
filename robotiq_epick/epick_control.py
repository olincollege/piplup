from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from pydrake.geometry import FrameId
import numpy as np
from suction_gripper import CupPressureSource, CupObjInterface


# Build Controls
def BuildEPickControl():
    pass


def AddSimEPickDriver(
    sim_plant: MultibodyPlant,
    suction_gripper_model: ModelInstanceIndex,
    obj_bodies: List[Body],
    builder: DiagramBuilder,
) -> System:
    inner_name = f"EPickDriver(epick_2cup)"
    inner_builder = DiagramBuilder()

    # TODO these constants need to be captured somewhere (krishna)
    kPumpPressure = -13e3
    kMaxSuctionDist = 0.004
    kNumSuctionCup = 2
    kSuctionModelTimeStep = 0.01
    kSuctionCupRadius = 0.026
    kSuctionCupArea = np.pi * (kSuctionCupRadius**2)

    suction_pressure_source = inner_builder.AddSystem(
        CupPressureSource(kPumpPressure, kMaxSuctionDist, kNumSuctionCup)
    )

    gripper_body = sim_plant.GetBodyByName(
        "robotiq_epick_2cup_base_link", suction_gripper_model
    )
    gripper_body_idx = gripper_body.index()
    action_point_frames = []
    edge_point_geoms = []
    for cup_idx in range(kNumSuctionCup):
        action_point_frames.append(
            sim_plant.GetFrameByName(
                f"suction_cup_{cup_idx}", suction_gripper_model
            ).GetFixedPoseInBodyFrame()
        )

        edge_point_geoms.append(set())
        for body_idx in sim_plant.GetBodyIndices(suction_gripper_model):
            body = sim_plant.get_body(body_idx)
            if f"cup_{cup_idx}_edge_pt" in body.name():
                edge_point_geom = sim_plant.GetCollisionGeometriesForBody(body)
                if len(edge_point_geom) != 1:
                    raise RuntimeError(
                        "Edge point has number of collision geometries not equal to 1"
                    )
                edge_point_geoms[cup_idx].add(edge_point_geom[0])

    obj_geom_id_to_body_idx_map = dict()
    for obj_body in obj_bodies:
        for obj_body_collision_geom in sim_plant.GetCollisionGeometriesForBody(
            obj_body
        ):
            obj_geom_id_to_body_idx_map[obj_body_collision_geom] = obj_body.index()

    gripper_frame_to_action_points = (
        sim_plant.GetBodyFrameIdOrThrow(gripper_body_idx),
        action_point_frames,
    )

    cup_obj_interface = inner_builder.AddSystem(
        CupObjInterface(
            kSuctionModelTimeStep,
            kSuctionCupArea,
            gripper_body_idx,
            gripper_frame_to_action_points,
            edge_point_geoms,
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

    inner_builder.ExportInput(
        suction_pressure_source.GetSuctionCmdInputPort(), "suction_command"
    )
    inner_builder.ExportInput(
        cup_obj_interface.GetGeomQueryInputPort(), "geometry_query"
    )
    inner_builder.ExportOutput(
        cup_obj_interface.GetSuctionForceOutputPort(), "suction_force"
    )

    diagram = inner_builder.Build()
    builder.AddNamedSystem(inner_name, diagram)
    return diagram
