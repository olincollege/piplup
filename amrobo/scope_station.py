import os
import sys
import warnings
from enum import Enum

import numpy as np
from pydrake.all import (
    AbstractValue,
    Adder,
    AddMultibodyPlantSceneGraph,
    BallRpyJoint,
    BaseField,
    Box,
    CameraInfo,
    Capsule,
    ClippingRange,
    CoulombFriction,
    Cylinder,
    Demultiplexer,
    DepthImageToPointCloud,
    DepthRange,
    DepthRenderCamera,
    DiagramBuilder,
    DifferentialInverseKinematicsIntegrator,
    DifferentialInverseKinematicsParameters,
    GeometryInstance,
    InverseDynamicsController,
    LeafSystem,
    MakePhongIllustrationProperties,
    MakeRenderEngineVtk,
    ModelInstanceIndex,
    MultibodyPlant,
    Parser,
    PassThrough,
    PrismaticJoint,
    RenderCameraCore,
    RenderEngineVtkParams,
    RevoluteJoint,
    Rgba,
    RgbdSensor,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    SpatialInertia,
    Sphere,
    StateInterpolatorWithDiscreteDerivative,
    UnitInertia,
)

from .utils import ConfigureParser

def MakeScopeStation(
    model_directives,
    time_step=0.002,
):
    builder = DiagramBuilder()

    # Add (only) the iiwa, WSG, and cameras to the scene.
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=time_step
    )
    parser = Parser(plant)
    ConfigureParser(parser)
    parser.AddModelsFromString(model_directives, ".dmd.yaml")
    plant.Finalize()

    for i in range(plant.num_model_instances()):
        model_instance = ModelInstanceIndex(i)
        model_instance_name = plant.GetModelInstanceName(model_instance)

        if model_instance_name.startswith("gen3"): # TODO real prefix 
            # TODO
            pass
        elif model_instance_name.startswith("2f_85"): # TODO real prefix
            pass
        elif model_instance_name.startswith("powerpick"): # TODO real prefix
            pass

    # Cameras.
    AddRgbdSensors(
        builder, plant, scene_graph
    )

    # Export "cheat" ports.
    builder.ExportOutput(scene_graph.get_query_output_port(), "query_object")
    builder.ExportOutput(
        plant.get_contact_results_output_port(), "contact_results"
    )
    builder.ExportOutput(
        plant.get_state_output_port(), "plant_continuous_state"
    )
    builder.ExportOutput(plant.get_body_poses_output_port(), "body_poses")

    diagram = builder.Build()
    diagram.set_name("AmroboStation")
    return diagram