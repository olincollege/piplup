from pydrake.all import *
from pydrake.multibody.plant import MultibodyPlant
from pydrake.geometry import (
    SceneGraph,
    DrakeVisualizer,
    DrakeVisualizerParams,
    Meshcat,
    MeshcatVisualizer,
)
from kinova_station.common import (
    EndEffectorTarget,
    GripperTarget,
    EndEffectorWrenchCalculator,
    CameraPosePublisher,
    ConfigureParser,
    EndEffectorType,
    JointTarget,
)
import os
import numpy as np


class KinovaStation(Diagram):
    """
                               ---------------------------------
                               |                               |
                               |                               | --> gen3.joint_commanded
                               |                               | --> gen3.joint_commanded_type
                               |                               |
                               |                               | --> gen3.measured_position
                               |                               | --> gen3.measured_velocity
    gen3.joint --------------> |         KinovaStation         | --> gen3.measured_torque
    gen3.joint_type ---------> |                               | --> gen3.external_torque
                               |                               |
                               |                               | --> gen3.ee_pose
                               |                               | --> gen3.ee_twist
                               |                               |
                               |                               | --> gripper.target_commanded
                               |                               | --> gripper.target_commanded_type
                               |                               |
    gripper.target ----------> |                               | --> gripper.measured_position
    gripper.target_type -----> |                               | --> gripper.measured_velocity
                               |                               |
    *visual.ee_target_pose* -> |                               | --> camera_[NAME].rgb_image
                               |                               | --> camera_[NAME].depth_image
                               |                               | --> *camera_[NAME].label_image*
                               |                               | --> *camera_[NAME].point_cloud*
                               |                               |
                               |                               | --> *query_object*
                               |                               | --> *contact_results*
                               |                               | --> *plant_continuous_state*
                               |                               | --> *geometry_poses*
                               |                               |
                               ---------------------------------
    Ports with ** are not available on hardware
    """

    def __init__(
        self,
        time_step=0.002,
        arm_damping=True,
        gripper_type=EndEffectorType.kRobotiq_2f_85,
    ):
        Diagram.__init__(self)
        self.set_name("kinova_manipulation_station")
        # TODO (krish-suresh) figure out how to specify multi-arm
        # TODO (krish-suresh) use the end effector type to setup the controller
        self.gripper_type: EndEffectorType = gripper_type

        self.builder = DiagramBuilder()

        self.scene_graph: SceneGraph = self.builder.AddSystem(SceneGraph())
        self.scene_graph.set_name("scene_graph")

        self.plant: MultibodyPlant = self.builder.AddSystem(
            MultibodyPlant(time_step=time_step)
        )
        self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        self.plant.set_name("plant")

        self.gen3 = AddGen3(self.plant, arm_damping)
        X_grip = RigidTransform()
        X_grip.set_rotation(RotationMatrix(RollPitchYaw([0, 0, np.pi / 2])))
        self.gripper = Add2f85(
            self.plant,
            self.plant.GetFrameByName("end_effector_frame", self.gen3),
            X_7G=X_grip,
            static=False,
        )

        # A separate plant which only has access to the robot arm + gripper mass,
        # and not any other objects in the scene
        self.controller_plant = MultibodyPlant(time_step=time_step)
        # TODO (krish-suresh) add static end effector model based on type for controller plant
        self.controller_gen3 = AddGen3(self.controller_plant, arm_damping)
        X_grip = RigidTransform()
        X_grip.set_rotation(RotationMatrix(RollPitchYaw([0, 0, np.pi / 2])))
        Add2f85(
            self.controller_plant,
            self.controller_plant.GetFrameByName(
                "end_effector_frame", self.controller_gen3
            ),
            X_7G=X_grip,
            static=True,
        )
        # Body id's and poses for any extra objects in the scene
        self.object_ids = []
        self.object_poses = []

    def Finalize(self):
        """
        Do some final setup stuff. Must be called after making modifications
        to the station (e.g. adding arm, gripper, manipulands) and before using
        this diagram as a system.
        """
        # match self.gripper_type:
        #     case EndEffectorType.kRobotiq_2f_85:
        #         # Add bushings to model the kinematic loop in the 2F-85 gripper.
        #         # This needs to be done pre-Finalize
        #         add_2f_85_bushings(self.plant, self.gripper)
        #     case EndEffectorType.kRobotiq_3f:
        #         pass
        #     case EndEffectorType.kRobotiqPowerPick:
        #         pass
        #     case EndEffectorType.kPunyo:
        #         pass

        self.plant.Finalize()
        self.controller_plant.Finalize()

        # Set up the scene graph
        self.builder.Connect(
            self.scene_graph.get_query_output_port(),
            self.plant.get_geometry_query_input_port(),
        )
        self.builder.Connect(
            self.plant.get_geometry_poses_output_port(),
            self.scene_graph.get_source_pose_port(self.plant.get_source_id()),
        )

        # pass through input ports to output
        joint_command: PassThrough = self.builder.AddSystem(PassThrough(7))
        self.builder.ExportInput(joint_command.get_input_port(), "gen3.joint")
        self.builder.ExportOutput(
            joint_command.get_output_port(), "gen3.joint_commanded"
        )

        joint_type_command: PassThrough = self.builder.AddSystem(
            PassThrough(AbstractValue.Make(JointTarget.kPosition))
        )
        self.builder.ExportInput(joint_type_command.get_input_port(), "gen3.joint_type")
        self.builder.ExportOutput(
            joint_type_command.get_output_port(), "gen3.joint_commanded_type"
        )

        # Output measured arm position and velocity
        demux: Demultiplexer = self.builder.AddSystem(
            Demultiplexer(
                self.plant.num_multibody_states(self.gen3),
                self.plant.num_positions(self.gen3),
            )
        )
        demux.set_name("demux")
        self.builder.Connect(
            self.plant.get_state_output_port(self.gen3), demux.get_input_port(0)
        )
        self.builder.ExportOutput(demux.get_output_port(0), "gen3.measured_position")
        self.builder.ExportOutput(demux.get_output_port(1), "gen3.measured_velocity")

        # Gen3 Arm controller
        gen3_controller: Gen3JointController = self.builder.AddSystem(
            Gen3JointController(self.controller_plant, self.controller_gen3)
        )
        gen3_controller.set_name("gen3_controller")
        self.builder.Connect(
            joint_command.get_output_port(),
            gen3_controller.GetInputPort("joint_target"),
        )
        self.builder.Connect(
            joint_type_command.get_output_port(),
            gen3_controller.GetInputPort("joint_target_type"),
        )
        self.builder.Connect(
            demux.get_output_port(0), gen3_controller.GetInputPort("arm_position")
        )
        self.builder.Connect(
            demux.get_output_port(1), gen3_controller.GetInputPort("arm_velocity")
        )

        self.builder.Connect(
            gen3_controller.GetOutputPort("applied_arm_torque"),
            self.plant.get_actuation_input_port(self.gen3),
        )
        self.builder.ExportOutput(
            gen3_controller.GetOutputPort("applied_arm_torque"), "gen3.measured_torque"
        )

        self.builder.ExportOutput(gen3_controller.GetOutputPort("measured_ee_pose"), "gen3.ee_pose")

        self.builder.ExportOutput(
            self.plant.get_generalized_contact_forces_output_port(self.gen3),
            "gen3.external_torque",
        )

        # Pass through gripper info
        gripper_target_command: PassThrough = self.builder.AddSystem(PassThrough(1))
        self.builder.ExportInput(
            gripper_target_command.get_input_port(), "gripper.target"
        )
        self.builder.ExportOutput(
            gripper_target_command.get_output_port(), "gripper.target_commanded"
        )

        gripper_target_type_command: PassThrough = self.builder.AddSystem(
            PassThrough(AbstractValue.Make(GripperTarget.kVelocity))
        )
        self.builder.ExportInput(
            gripper_target_type_command.get_input_port(), "gripper.target_type"
        )
        self.builder.ExportOutput(
            gripper_target_type_command.get_output_port(),
            "gripper.target_commanded_type",
        )

        # Create gripper controller
        gripper_controller = self.builder.AddSystem(GripperController())
        gripper_controller.set_name("gripper_controller")

        # Connect gripper controller to the diagram
        self.builder.Connect(
            gripper_target_command.get_output_port(),
            gripper_controller.GetInputPort("gripper_target"),
        )
        self.builder.Connect(
            gripper_target_type_command.get_output_port(),
            gripper_controller.GetInputPort("gripper_target_type"),
        )

        self.builder.Connect(
            self.plant.get_state_output_port(self.gripper),
            gripper_controller.GetInputPort("gripper_state"),
        )
        self.builder.Connect(
            gripper_controller.GetOutputPort("applied_gripper_torque"),
            self.plant.get_actuation_input_port(self.gripper),
        )

        # Send gripper position and velocity as an output
        self.builder.ExportOutput(
            gripper_controller.GetOutputPort("measured_gripper_position"),
            "gripper.measured_position",
        )
        self.builder.ExportOutput(
            gripper_controller.GetOutputPort("measured_gripper_velocity"),
            "gripper.measured_velocity",
        )

        # TODO add camera output port for each camera
        # for camera in cameras:
        # Create and add the camera system
        # camera_parent_body_id = self.plant.GetBodyFrameIdIfExists(
        #     self.camera_parent_frame.body().index()
        # )
        # camera = self.builder.AddSystem(
        #     RgbdSensor(
        #         camera_parent_body_id,
        #         self.X_camera,
        #         self.color_camera,
        #         self.depth_camera,
        #     )
        # )
        # camera.set_name("camera")

        # # Wire the camera to the scene graph
        # self.builder.Connect(
        #     self.scene_graph.get_query_output_port(),
        #     camera.query_object_input_port(),
        # )

        # self.builder.ExportOutput(
        #     camera.color_image_output_port(), "camera_[NAME].rgb_image"
        # )
        # self.builder.ExportOutput(
        #     camera.depth_image_32F_output_port(), "camera_[NAME].depth_image"
        # )
        # self.builder.ExportOutput(
        #     # TODO (),
        #     "camera_[NAME].label_image",
        # )
        # self.builder.ExportOutput(
        #     # TODO (),
        #     "camera_[NAME].point_cloud",
        # )

        # Cheat ports
        self.builder.ExportOutput(
            self.scene_graph.get_query_output_port(), "query_object"
        )
        self.builder.ExportOutput(
            self.plant.get_contact_results_output_port(), "contact_results"
        )
        self.builder.ExportOutput(
            self.plant.get_state_output_port(), "plant_continuous_state"
        )
        self.builder.ExportOutput(
            self.plant.get_body_poses_output_port(), "geometry_poses"
        )

        # Build the diagram
        self.builder.BuildInto(self)

    def AddGround(self):
        """
        Add a flat ground with friction
        """
        X_BG = RigidTransform()
        surface_friction = CoulombFriction(static_friction=0.7, dynamic_friction=0.5)
        self.plant.RegisterCollisionGeometry(
            self.plant.world_body(),
            X_BG,
            HalfSpace(),
            "ground_collision",
            surface_friction,
        )
        self.plant.RegisterVisualGeometry(
            self.plant.world_body(),
            X_BG,
            HalfSpace(),
            "ground_visual",
            np.array([0.5, 0.5, 0.5, 0]),
        )  # transparent

    def AddManipulandFromFile(self, model_file, X_WObject):
        """
        Add an object to the simulation and place it in the given pose in the world
        """
        parser = Parser(plant=self.plant)
        ConfigureParser(parser)
        manipuland = parser.AddModelFromFile(model_file)
        body_indices = self.plant.GetBodyIndices(manipuland)

        assert len(body_indices) == 1, "Only single-body objects are supported for now"

        self.object_ids.append(body_indices[0])
        self.object_poses.append(X_WObject)

    def SetArmPositions(self, diagram, diagram_context, q):
        """
        Set arm positions to the given values. Must be called after the overall
        system diagram is built, and the associated diagram_context set.
        """
        plant_context = diagram.GetMutableSubsystemContext(self.plant, diagram_context)
        self.plant.SetPositions(plant_context, self.gen3, q)

    def SetManipulandStartPositions(self, diagram, diagram_context):
        """
        Set positions of any manipulands to their starting values. Must be called
        after the overall system diagram is built, and the associated diagram_context set.
        """
        assert len(self.object_ids) == len(
            self.object_poses
        ), "Manipuland poses and ids don't match"

        plant_context = diagram.GetMutableSubsystemContext(self.plant, diagram_context)

        for i in range(len(self.object_ids)):
            self.plant.SetFreeBodyPose(
                plant_context,
                self.plant.get_body(self.object_ids[i]),
                self.object_poses[i],
            )

    def ConnectToDrakeVisualizer(self):
        visualizer_params = DrakeVisualizerParams(role=Role.kIllustration)
        DrakeVisualizer().AddToBuilder(
            builder=self.builder, scene_graph=self.scene_graph, params=visualizer_params
        )

    def ConnectToMeshcatVisualizer(self, port=None):
        self.meshcat = Meshcat(port)
        mcpp = MeshcatVisualizer(self.meshcat)
        mcpp.AddToBuilder(self.builder, self.scene_graph, self.meshcat)

        print(
            "Open %s in a browser to view the meshcat visualizer."
            % self.meshcat.web_url()
        )

    def go_home(self, diagram, diagram_context, name="Home"):
        """
        Move the arm to the specified home position. Must be one of
        'Home', 'Retract', or 'Zero'.
        """
        if name == "Home":
            q0 = np.array(
                [0, np.pi / 12, np.pi, 4.014 - 2 * np.pi, 0, 0.9599, np.pi / 2]
            )
        elif name == "Retract":
            q0 = np.array(
                [
                    0,
                    5.93 - 2 * np.pi,
                    np.pi,
                    3.734 - 2 * np.pi,
                    0,
                    5.408 - 2 * np.pi,
                    np.pi / 2,
                ]
            )
        elif name == "Zero":
            q0 = np.zeros(7)
        else:
            raise RuntimeError(
                "Home position name must be one of ['Home', 'Retract', 'Zero']"
            )

        self.SetArmPositions(diagram, diagram_context, q0)


class Gen3JointController(LeafSystem):
    """
    A controller which imitates the cartesian control mode of the Kinova gen3 arm.

                         -------------------------
                         |                       |
                         |                       |
    joint_target ------> |  Gen3JointController  | ----> applied_arm_torque
    joint_target_type -> |                       |
                         |                       |
                         |                       |
    arm_position ------> |                       |
    arm_velocity ------> |                       |
                         |                       |
                         |                       |
                         -------------------------
    """

    def __init__(self, plant: MultibodyPlant, arm_model):
        LeafSystem.__init__(self)

        self.plant = plant
        self.arm = arm_model
        self.context = self.plant.CreateDefaultContext()

        # Define input ports
        self.joint_target_port = self.DeclareVectorInputPort(
            "joint_target", BasicVector(self.plant.num_actuators())
        )
        self.joint_target_type_port = self.DeclareAbstractInputPort(
            "joint_target_type", AbstractValue.Make(JointTarget.kPosition)
        )

        self.arm_position_port = self.DeclareVectorInputPort(
            "arm_position", BasicVector(self.plant.num_positions(self.arm))
        )
        self.arm_velocity_port = self.DeclareVectorInputPort(
            "arm_velocity", BasicVector(self.plant.num_velocities(self.arm))
        )
        self.DeclareDiscreteState(7)
        # Define output ports
        self.DeclareVectorOutputPort(
            "applied_arm_torque",
            BasicVector(self.plant.num_actuators()),
            self.CalcArmTorques,
        )

        self.DeclareVectorOutputPort(
                "measured_ee_pose",
                BasicVector(7),
                self.CalcEndEffectorPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep
        self.DeclareVectorOutputPort(
                "measured_ee_twist",
                BasicVector(6),
                self.CalcEndEffectorTwist,
                {self.time_ticket()})
        # Define some relevant frames
        self.world_frame = self.plant.world_frame()
        self.ee_frame = self.plant.GetFrameByName("end_effector_frame")


    def get_multibody_plant_for_control(self):
        return self.plant

    def CalcEndEffectorPose(self, context, output):
        """
        This method is called each timestep to determine the end-effector pose
        """
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
        self.plant.SetPositions(self.context,q)
        self.plant.SetVelocities(self.context,qd)

        # Compute the rigid transform between the world and end-effector frames
        X_ee : RigidTransform= self.plant.CalcRelativeTransform(self.context,
                                                self.world_frame,
                                                self.ee_frame)
        ee_pose = np.hstack([X_ee.rotation().ToQuaternion().wxyz(), X_ee.translation()])

        output.SetFromVector(ee_pose)
    
    def CalcEndEffectorTwist(self, context, output):
        """
        This method is called each timestep to determine the end-effector twist
        """
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
        self.plant.SetPositions(self.context,q)
        self.plant.SetVelocities(self.context,qd)

        # Compute end-effector Jacobian
        J = self.plant.CalcJacobianSpatialVelocity(self.context,
                                                   JacobianWrtVariable.kV,
                                                   self.ee_frame,
                                                   np.zeros(3),
                                                   self.world_frame,
                                                   self.world_frame)

        ee_twist = J@qd
        output.SetFromVector(ee_twist)
    def CalcArmTorques(self, context:Context, output):
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
        self.plant.SetPositions(self.context, q)
        self.plant.SetVelocities(self.context, qd)

        # Some dynamics computations
        tau_g = -self.plant.CalcGravityGeneralizedForces(self.context)

        # Indicate what type of command we're recieving
        target_type = self.joint_target_type_port.Eval(context)

        if target_type == JointTarget.kTorque:
            tau = tau_g + self.joint_target_port.Eval(context)

        elif target_type == JointTarget.kVelocity:
            # Compue joint torques consistent with the desired twist
            qd_nom = self.joint_target_port.Eval(context)

            # Select desired accelerations using a proportional controller
            Kp = 1000 * np.eye(self.plant.num_velocities())
            qdd_nom = Kp @ (qd_nom - qd)

            # Compute joint torques consistent with these desired accelerations
            f_ext = MultibodyForces(self.plant)
            tau = tau_g + self.plant.CalcInverseDynamics(self.context, qdd_nom, f_ext)

        elif target_type == JointTarget.kPosition:
            # Compute joint torques which move the end effector to the desired pose
            q_nom = self.joint_target_port.Eval(context)

            qd_nom = np.zeros(self.plant.num_velocities())

            # Use PD controller to map desired q, qd to desired qdd
            Kp = 100 * np.eye(self.plant.num_positions())
            Ki = 0.1 * np.eye(self.plant.num_positions())
            Kd = 10 * np.eye(self.plant.num_positions())
            context.SetDiscreteState(0, context.get_discrete_state(0).value() + Ki@(q_nom - q))
            qdd_nom = Kp @ (q_nom - q) + context.get_discrete_state(0).value() + Kd @ (qd_nom - qd)

            # Compute joint torques consistent with these desired qdd
            f_ext = MultibodyForces(self.plant)
            tau = tau_g + self.plant.CalcInverseDynamics(self.context, qdd_nom, f_ext)

        else:
            raise RuntimeError("Invalid target type %s" % target_type)

        output.SetFromVector(tau)


class GripperController(LeafSystem):
    """
    A simple gripper controller with two modes: position and velocity.
    Both modes are essentially simple PD controllers.

                            -------------------------
                            |                       |
                            |                       |
    gripper_target -------> |   GripperController   | ---> applied_gripper_torques
    gripper_target_type --> |                       |
                            |                       | ---> measured_gripper_position
                            |                       | ---> measured_gripper_velocity
    gripper_state --------> |                       |
                            |                       |
                            |                       |
                            -------------------------
    """

    def __init__(self):
        """
        The type can be "hande" or "2f_85", depending on the type of gripper.
        """
        LeafSystem.__init__(self)

        state_size = 12

        # We'll create a simple model of the gripper which is welded to the floor.
        # This will allow us to compute the distance between fingers.
        self.plant = MultibodyPlant(1.0)  # timestep doesn't matter
        self.gripper = Add2f85(self.plant, self.plant.world_frame())
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # Declare input ports
        self.target_port = self.DeclareVectorInputPort("gripper_target", BasicVector(1))
        self.target_type_port = self.DeclareAbstractInputPort(
            "gripper_target_type", AbstractValue.Make(GripperTarget.kPosition)
        )

        self.state_port = self.DeclareVectorInputPort(
            "gripper_state", BasicVector(state_size)
        )

        # Declare output ports
        self.DeclareVectorOutputPort(
            "applied_gripper_torque", BasicVector(2), self.CalcGripperTorque
        )
        self.DeclareVectorOutputPort(
            "measured_gripper_position",
            BasicVector(1),
            self.CalcGripperPosition,
            {self.time_ticket()},  # indicate that this doesn't depend on any inputs,
        )  # but should still be updated each timestep
        self.DeclareVectorOutputPort(
            "measured_gripper_velocity",
            BasicVector(1),
            self.CalcGripperVelocity,
            {self.time_ticket()},
        )

    def ComputePosition(self, state):
        """
        Compute the gripper position from state data.
        This is especially useful for the 2F-85 gripper, since the
        state does not map neatly to the finger positions.
        """
        # For the more complex 2F-85 gripper, we need to do some kinematics
        # calculations to figure out the gripper position
        self.plant.SetPositionsAndVelocities(self.context, state)

        right_finger = self.plant.GetFrameByName("right_inner_finger_pad")
        left_finger = self.plant.GetFrameByName("left_inner_finger_pad")
        base = self.plant.GetFrameByName("robotiq_arg2f_base_link")

        X_lf = self.plant.CalcRelativeTransform(self.context, left_finger, base)
        X_rf = self.plant.CalcRelativeTransform(self.context, right_finger, base)

        lf_pos = -X_lf.translation()[1]
        rf_pos = -X_rf.translation()[1]

        finger_position = np.array([lf_pos, rf_pos])

        return finger_position

    def ComputeVelocity(self, state):
        """
        Compute the gripper velocity from state data.
        This is especially useful for the 2F-85 gripper, since the
        state does not map neatly to the finger positions.
        """
        # For the more complex 2F-85 gripper, we need to do some kinematics
        self.plant.SetPositionsAndVelocities(self.context, state)
        v = state[-self.plant.num_velocities() :]

        right_finger = self.plant.GetFrameByName("right_inner_finger_pad")
        left_finger = self.plant.GetFrameByName("left_inner_finger_pad")
        base = self.plant.GetFrameByName("robotiq_arg2f_base_link")

        J_lf = self.plant.CalcJacobianTranslationalVelocity(
            self.context,
            JacobianWrtVariable.kV,
            left_finger,
            np.zeros(3),
            base,
            base,
        )
        J_rf = self.plant.CalcJacobianTranslationalVelocity(
            self.context,
            JacobianWrtVariable.kV,
            right_finger,
            np.zeros(3),
            base,
            base,
        )

        lf_vel = -(J_lf @ v)[1]
        rf_vel = (J_rf @ v)[1]

        finger_velocity = np.array([lf_vel, rf_vel])

        return finger_velocity

    def CalcGripperPosition(self, context, output):
        state = self.state_port.Eval(context)

        width = 0.06

        # Send a single number to match the hardware
        both_finger_positions = self.ComputePosition(state)
        net_position = 1 / width * np.mean(both_finger_positions)

        output.SetFromVector([net_position])

    def CalcGripperVelocity(self, context, output):
        state = self.state_port.Eval(context)

        width = 0.06

        # Send a single number to match the hardware
        both_finger_velocity = self.ComputeVelocity(state)
        net_velocity = 1 / width * np.mean(both_finger_velocity)

        output.SetFromVector([net_velocity])

    def CalcGripperTorque(self, context, output):
        state = self.state_port.Eval(context)
        target = self.target_port.Eval(context)
        target_type = self.target_type_port.Eval(context)

        finger_position = self.ComputePosition(state)
        finger_velocity = self.ComputeVelocity(state)

        width = 0.06
        Kp = 10 * np.eye(2)
        Kd = 2 * np.sqrt(0.01 * Kp)

        # Set target positions and velocities based on the current control mode
        if target_type == GripperTarget.kPosition:
            target = width - width * target * np.ones(2)
            target_finger_position = target
            target_finger_velocity = np.zeros(2)
        elif target_type == GripperTarget.kVelocity:
            target_finger_position = finger_position
            target_finger_velocity = -width * target
        else:
            raise RuntimeError("Invalid gripper target type: %s" % target_type)

        # Determine applied torques with PD controller
        position_err = target_finger_position - finger_position
        velocity_err = target_finger_velocity - finger_velocity
        tau = -Kp @ (position_err) - Kd @ (velocity_err)

        output.SetFromVector(tau)


def AddPowerPick(plant, frame_to_weld_gripper_base, X_7G=RigidTransform()):
    pass  # TODO (krish-suresh)


def Add3f(plant, frame_to_weld_gripper_base, X_7G=RigidTransform(), static=False):
    pass  # TODO (krish-suresh)


def Add2f85(plant, frame_to_weld_gripper_base, X_7G=RigidTransform(), static=False):
    parser = Parser(plant)
    ConfigureParser(parser)
    gripper = parser.AddModelsFromUrl(
        f"package://amrobo/robotiq_description/sdf/robotiq_2f_85{'_static' if static else ''}.sdf"
    )[0]
    plant.WeldFrames(
        frame_to_weld_gripper_base,
        plant.GetFrameByName("robotiq_arg2f_base_link", gripper),
        X_7G,
    )
    return gripper


def AddGen3(plant: MultibodyPlant, include_damping=False):
    parser = Parser(plant)
    ConfigureParser(parser)
    if include_damping:
        # The hardware system has lots of damping so this is more realistic,
        # but requires a simulation with small timesteps.
        arm_url = (
            "package://amrobo/gen3_description/sdf/gen3_mesh_collision_with_damping.sdf"
        )
    else:
        arm_url = "package://amrobo/gen3_description/sdf/gen3_mesh_collision.sdf"

    gen3 = parser.AddModelsFromUrl(arm_url)[0]

    X_ee = RigidTransform()
    X_ee.set_translation([0, 0, -0.0615250000000001])
    X_ee.set_rotation(RotationMatrix(RollPitchYaw([np.pi, 0, 0])))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link", gen3))
    plant.AddFrame(
        FixedOffsetFrame(
            "end_effector_frame",
            plant.GetFrameByName("bracelet_no_vision_link"),
            X_ee,
            gen3,
        )
    )
    return gen3