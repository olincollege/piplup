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
    LinearBushingRollPitchYaw,
    FixedOffsetFrame,
    Diagram
)

from .utils import ConfigureParser


class ScopeStation(Diagram):


class ExtractBodyPose(LeafSystem):
    def __init__(self, body_poses_output_port, body_index):
        LeafSystem.__init__(self)
        self.body_index = body_index
        self.DeclareAbstractInputPort(
            "poses",
            body_poses_output_port.Allocate(),
        )
        self.DeclareAbstractOutputPort(
            "pose",
            lambda: AbstractValue.Make(RigidTransform()),
            self.CalcOutput,
        )

    def CalcOutput(self, context, output):
        poses = self.EvalAbstractInput(context, 0).get_value()
        pose = poses[int(self.body_index)]
        output.get_mutable_value().set(pose.rotation(), pose.translation())

def AddRgbdSensors(
    builder,
    plant,
    scene_graph,
    also_add_point_clouds=True,
    depth_camera=None,
    renderer=None,
):
    """
    Adds a RgbdSensor to the first body in the plant for every model instance
    with a name starting with model_instance_prefix.  If depth_camera is None,
    then a default camera info will be used.  If renderer is None, then we will
    assume the name 'my_renderer', and create a VTK renderer if a renderer of
    that name doesn't exist.
    """
    if sys.platform == "linux" and os.getenv("DISPLAY") is None:
        from pyvirtualdisplay import Display

        virtual_display = Display(visible=0, size=(1400, 900))
        virtual_display.start()

    if not renderer:
        renderer = "my_renderer"

    if not scene_graph.HasRenderer(renderer):
        scene_graph.AddRenderer(
            renderer, MakeRenderEngineVtk(RenderEngineVtkParams())
        )

    if not depth_camera:
        depth_camera = DepthRenderCamera(
            RenderCameraCore(
                renderer,
                CameraInfo(width=640, height=480, fov_y=np.pi / 4.0),
                ClippingRange(near=0.1, far=10.0),
                RigidTransform(),
            ),
            DepthRange(0.1, 10.0),
        )

    for index in range(plant.num_model_instances()):
        model_instance_index = ModelInstanceIndex(index)
        model_name = plant.GetModelInstanceName(model_instance_index)

        if model_name.startswith("camera"):
            body_index = plant.GetBodyIndices(model_instance_index)[0]
            rgbd = builder.AddSystem(
                RgbdSensor(
                    parent_id=plant.GetBodyFrameIdOrThrow(body_index),
                    X_PB=RigidTransform(),
                    depth_camera=depth_camera,
                    show_window=False,
                )
            )
            rgbd.set_name(model_name)

            builder.Connect(
                scene_graph.get_query_output_port(),
                rgbd.query_object_input_port(),
            )

            # Export the camera outputs
            builder.ExportOutput(
                rgbd.color_image_output_port(), f"{model_name}_rgb_image"
            )
            builder.ExportOutput(
                rgbd.depth_image_32F_output_port(), f"{model_name}_depth_image"
            )
            builder.ExportOutput(
                rgbd.label_image_output_port(), f"{model_name}_label_image"
            )

            if also_add_point_clouds:
                # Add a system to convert the camera output into a point cloud
                to_point_cloud = builder.AddSystem(
                    DepthImageToPointCloud(
                        camera_info=rgbd.depth_camera_info(),
                        fields=BaseField.kXYZs | BaseField.kRGBs,
                    )
                )
                builder.Connect(
                    rgbd.depth_image_32F_output_port(),
                    to_point_cloud.depth_image_input_port(),
                )
                builder.Connect(
                    rgbd.color_image_output_port(),
                    to_point_cloud.color_image_input_port(),
                )

                camera_pose = builder.AddSystem(
                    ExtractBodyPose(
                        plant.get_body_poses_output_port(), body_index
                    )
                )
                builder.Connect(
                    plant.get_body_poses_output_port(),
                    camera_pose.get_input_port(),
                )
                builder.Connect(
                    camera_pose.get_output_port(),
                    to_point_cloud.GetInputPort("camera_pose"),
                )

                # Export the point cloud output.
                builder.ExportOutput(
                    to_point_cloud.point_cloud_output_port(),
                    f"{model_name}_point_cloud",
                )


# TODO maybe move this to some parsing class
def add_2f_85_bushings(plant, gripper):
    """
    The Robotiq 2F-85 gripper has a complicated mechanical component which includes
    a kinematic loop. The original URDF deals with this by using a "mimic" tag,
    but Drake doesn't support this. So we'll try to close the loop using a bushing,
    as described in Drake's four-bar linkage example: 

        https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/four_bar.

    This needs to be done before plant is finalized. 
    """
    left_inner_finger = plant.GetFrameByName("left_inner_finger", gripper)
    left_inner_knuckle = plant.GetFrameByName("left_inner_knuckle", gripper)
    right_inner_finger = plant.GetFrameByName("right_inner_finger", gripper)
    right_inner_knuckle = plant.GetFrameByName("right_inner_knuckle", gripper)

    # Add frames which are located at the desired linkage point
    X_finger = RigidTransform()
    X_finger.set_translation([0.0,-0.016,0.007])
    X_knuckle = RigidTransform()
    X_knuckle.set_translation([0.0,0.038,0.043])

    left_inner_finger_bushing = FixedOffsetFrame(
                                        "left_inner_finger_bushing",
                                        left_inner_finger,
                                        X_finger,
                                        gripper)
    left_inner_knuckle_bushing = FixedOffsetFrame(
                                        "left_inner_knuckle_bushing",
                                        left_inner_knuckle,
                                        X_knuckle,
                                        gripper)
    right_inner_finger_bushing = FixedOffsetFrame(
                                        "right_inner_finger_bushing",
                                        right_inner_finger,
                                        X_finger,
                                        gripper)
    right_inner_knuckle_bushing = FixedOffsetFrame(
                                        "right_inner_knuckle_bushing",
                                        right_inner_knuckle,
                                        X_knuckle,
                                        gripper)


    plant.AddFrame(left_inner_finger_bushing)
    plant.AddFrame(left_inner_knuckle_bushing)
    plant.AddFrame(right_inner_finger_bushing)
    plant.AddFrame(right_inner_knuckle_bushing)

    # Force and torque stiffness and damping describe a revolute joint on the z-axis
    k_xyz = 8000
    d_xyz = 10
    k_rpy = 15
    d_rpy = 3
    force_stiffness_constants =  np.array([k_xyz,k_xyz,k_xyz])
    force_damping_constants =    np.array([d_xyz,d_xyz,d_xyz])
    torque_stiffness_constants = np.array([0,k_rpy,k_rpy])
    torque_damping_constants =   np.array([0,d_rpy,k_rpy])

    left_finger_bushing = LinearBushingRollPitchYaw(
                left_inner_finger_bushing, left_inner_knuckle_bushing,
                torque_stiffness_constants, torque_damping_constants,
                force_stiffness_constants, force_damping_constants)
    right_finger_bushing = LinearBushingRollPitchYaw(
                right_inner_finger_bushing, right_inner_knuckle_bushing,
                torque_stiffness_constants, torque_damping_constants,
                force_stiffness_constants, force_damping_constants)
    plant.AddForceElement(left_finger_bushing)
    plant.AddForceElement(right_finger_bushing)



def MakeScopeStation(
    model_directives,
    time_step=0.002,
):
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=time_step
    )
    parser = Parser(plant)
    ConfigureParser(parser)
    parser.AddModelsFromString(model_directives, ".dmd.yaml")
    # TODO if 2f gripper: add_2f_85_bushings(self.plant, self.gripper)
    plant.Finalize()

    for i in range(plant.num_model_instances()):
        model_instance = ModelInstanceIndex(i)
        model_instance_name = plant.GetModelInstanceName(model_instance)

        if model_instance_name.startswith("gen3"): # TODO real prefix 
            # TODO Inverse Dynamics Controller
            pass
        elif model_instance_name.startswith("2f_85"): # TODO real prefix
            # TODO Gripper PD Controller
            pass
        elif model_instance_name.startswith("powerpick"): # TODO real prefix
            # TODO some form of boolean suction
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
    diagram.set_name("Station")
    return diagram