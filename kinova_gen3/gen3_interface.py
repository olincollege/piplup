import threading

from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import (
    DeviceConfig_pb2,
    Session_pb2,
    Base_pb2,
    VisionConfig_pb2,
)
import sys
from .gen3_constants import (
    Gen3ControlMode,
    kGen3ArmNumJoints,
    Gen3NamedPosition,
    kGen3NamedPositions,
)

from common import *


class Gen3InterfaceConfig:
    __fields__: ClassVar[tuple] = (
        SimpleNamespace(name="port", type=str),
        SimpleNamespace(name="ip_address", type=str),
    )
    port: str
    ip_address: str


class Gen3HardwareInterface(LeafSystem):
    def __init__(self, ip_address, port, hand_model_name, sim_plant: MultibodyPlant):
        LeafSystem.__init__(self)
        self.hand_model_name = hand_model_name
        self.sim_plant = sim_plant
        # World to robot base
        self.X_WB: RigidTransform = self.sim_plant.CalcRelativeTransform(
            self.sim_plant.CreateDefaultContext(),
            self.sim_plant.world_frame(),
            self.sim_plant.GetFrameByName(
                "base_link", self.sim_plant.GetModelInstanceByName("gen3")
            ),
        )
        self.X_WB_inv: RigidTransform = self.X_WB.inverse()

        self.root_ctx = None

        # 7 values are one of the following:
        #   joint positions
        #   joint velocities
        #   pose (rpy, xyz)
        #   twist (rpy, xyz, 0)
        self.arm_command_input_port = self.DeclareVectorInputPort("command", 7)
        self.control_mode_input_port = self.DeclareAbstractInputPort(
            "control_mode", AbstractValue.Make(Gen3ControlMode.kPose)
        )

        if hand_model_name == "2f_85":
            self.gripper_command_input_port = self.DeclareVectorInputPort(
                "2f_85.command", 1
            )
        # self.DeclareVectorOutputPort(
        #     "position_measured", BasicVector(kGen3ArmNumJoints), self.CalcArmPosition
        # )
        # self.DeclareVectorOutputPort(
        #     "velocity_measured", BasicVector(kGen3ArmNumJoints), self.CalcArmVelocity
        # )
        # self.DeclareVectorOutputPort(
        #     "torque_measured", BasicVector(kGen3ArmNumJoints), self.CalcArmTorque
        # )
        # self.DeclareVectorOutputPort(
        #     "state_estimated", BasicVector(kGen3ArmNumJoints * 2), self.CalcState
        # )
        self.DeclareVectorOutputPort(
            "pose_measured",
            BasicVector(6),
            self.CalcEndEffectorPose,
            {self.time_ticket()},
        )
        self.DeclarePeriodicUnrestrictedUpdateEvent(1 / 40, 0, self.Integrate)

        self.last_feedback_time = -np.inf
        self.feedback = None

        logging.info("Connecting to gen3 arm...")
        self.transport = TCPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
        self.transport.connect(ip_address, int(port))
        # Create session
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = "admin"
        session_info.password = "admin"
        session_info.session_inactivity_timeout = 6000  # (milliseconds)
        session_info.connection_inactivity_timeout = 2000  # (milliseconds)

        self.session_manager = SessionManager(self.router)
        self.session_manager.CreateSession(session_info)

        # Create required services
        device_config = DeviceConfigClient(self.router)
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)

        if self.base.GetArmState().active_state != Base_pb2.ARMSTATE_SERVOING_READY:
            logging.error(self.base.GetArmState())
            raise RuntimeError(
                "Arm not in ready state. Clear any faults before trying again."
            )

    def CleanUp(self):
        logging.info("\nClosing Hardware Connection...")

        if self.session_manager is not None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000

            self.session_manager.CloseSession()

        self.transport.disconnect()
        logging.info("Hardware Connection Closed.")

    def GetFeedback(self, current_time):
        """
        Sets self.feedback with the latest data from the controller.
        We also indicate the time at which this feedback was set, in an
        effor to reduce redudnant calls to self.base_cyclic.RefreshFeedback(),
        which take about 25ms each.
        """
        self.feedback = self.base_cyclic.RefreshFeedback()
        self.last_feedback_time = current_time

    def check_for_end_or_abort(self, e):
        """
        Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """

        def check(notification, e=e):
            if (
                notification.action_event == Base_pb2.ACTION_END
                or notification.action_event == Base_pb2.ACTION_ABORT
            ):
                e.set()

        return check

    def go_to_named_position(self, named_position: Gen3NamedPosition):
        self.SendPositionCommand(kGen3NamedPositions[named_position])

    def SendPoseCommand(self, position: np.ndarray, rpy: RollPitchYaw):
        action = Base_pb2.Action()
        action.name = "End-effector pose command"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.theta_x = np.degrees(rpy.roll_angle())
        cartesian_pose.theta_y = np.degrees(rpy.pitch_angle())
        cartesian_pose.theta_z = np.degrees(rpy.yaw_angle())
        cartesian_pose.x = position[0]
        cartesian_pose.y = position[1]
        cartesian_pose.z = position[2]

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e), Base_pb2.NotificationOptions()
        )

        self.base.ExecuteAction(action)

        TIMEOUT_DURATION = 20  # seconds
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

    def SendTwistCommand(self, cmd_twist: np.ndarray):
        command = Base_pb2.TwistCommand()
        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        command.duration = 0
        twist = command.twist
        twist.angular_x = np.degrees(cmd_twist[0])
        twist.angular_y = np.degrees(cmd_twist[1])
        twist.angular_z = np.degrees(cmd_twist[2])
        twist.linear_x = cmd_twist[3]
        twist.linear_y = cmd_twist[4]
        twist.linear_z = cmd_twist[5]

        # Note: this API call takes about 25ms
        self.base.SendTwistCommand(command)

    def SendWrenchCommand(self, cmd_wrench):
        command = Base_pb2.WrenchCommand()
        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        command.duration = 0

        wrench = command.wrench
        wrench.torque_x = cmd_wrench[0]
        wrench.torque_y = cmd_wrench[1]
        wrench.torque_z = cmd_wrench[2]
        wrench.force_x = cmd_wrench[3]
        wrench.force_y = cmd_wrench[4]
        wrench.force_z = cmd_wrench[5]

        self.base.SendWrenchCommand(command)

    def SendPositionCommand(self, joint_positions: np.ndarray):
        action = Base_pb2.Action()
        action.name = ""
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

        # Place arm straight up
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = np.degrees(joint_positions[joint_id])

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e), Base_pb2.NotificationOptions()
        )

        self.base.ExecuteAction(action)
        TIMEOUT_DURATION = 20  # seconds
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

    def SendVelocityCommand(self, joint_velocities: np.ndarray):
        joint_speeds = Base_pb2.JointSpeeds()

        actuator_count = self.base.GetActuatorCount().count
        assert actuator_count == len(joint_velocities)
        for i, speed in enumerate(joint_velocities):
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i
            joint_speed.value = speed
            joint_speed.duration = 0

        self.base.SendJointSpeedsCommand(joint_speeds)

    def Integrate(self, context: Context, discrete_state: DiscreteValues):
        # def DoCalcTimeDerivatives(self, context, continuous_state):
        # TODO this is a bad way to do this:
        ## ------
        t = context.get_time()
        if self.last_feedback_time != t:
            self.GetFeedback(t)

        q = np.zeros(kGen3ArmNumJoints)
        for i in range(kGen3ArmNumJoints):
            q[i] = np.radians(self.feedback.actuators[i].position)

        self.sim_plant.SetPositions(
            self.sim_plant.GetMyMutableContextFromRoot(self.root_ctx),
            self.sim_plant.GetModelInstanceByName("gen3"),
            q,
        )
        self.sim_plant.SetPositions(
            self.sim_plant.GetMyMutableContextFromRoot(self.root_ctx),
            self.sim_plant.GetModelInstanceByName("2f_85"),
            np.zeros(6),
        )
        ## ------
        if self.hand_model_name == "2f_85":
            gripper_command = Base_pb2.GripperCommand()
            gripper_command.mode = Base_pb2.GRIPPER_SPEED

            finger = gripper_command.gripper.finger.add()
            finger.finger_identifier = 1
            finger.value = self.gripper_command_input_port.Eval(context)[0]

            self.base.SendGripperCommand(gripper_command)

        control_mode: Gen3ControlMode = self.control_mode_input_port.Eval(context)

        command: BasicVector = self.arm_command_input_port.Eval(context)

        match control_mode:
            case Gen3ControlMode.kPosition:
                self.SendPositionCommand(command)
            case Gen3ControlMode.kVelocity:
                self.SendVelocityCommand(command)
            case Gen3ControlMode.kPose:
                translation = command[3:-1]
                rpy = RollPitchYaw(command[:3])
                # X_WE = X_WB @ X_BE
                X_BE = self.X_WB_inv = RigidTransform(rpy.ToQuaternion(), translation)
                self.SendPoseCommand(
                    X_BE.translation(), X_BE.rotation().ToRollPitchYaw()
                )
            case Gen3ControlMode.kTwist:
                self.SendTwistCommand(command[:6])
            case _:
                raise RuntimeError(f"Unsupported control mode {control_mode}")

    def CalcArmPosition(self, context, output):
        """
        Compute the current joint angles and send as output.
        """
        # Get feedback from the base, but only if we haven't already this timestep
        t = context.get_time()
        if self.last_feedback_time != t:
            self.GetFeedback(t)

        q = np.zeros(kGen3ArmNumJoints)
        for i in range(kGen3ArmNumJoints):
            q[i] = np.radians(
                self.feedback.actuators[i].position
            )  # Kortex provides joint angles
            # in degrees for some reason
        output.SetFromVector(q)

    def CalcArmVelocity(self, context, output):
        """
        Compute the current joint velocities and send as output.
        """
        t = context.get_time()
        if self.last_feedback_time != t:
            self.GetFeedback(t)

        qd = np.zeros(kGen3ArmNumJoints)
        for i in range(kGen3ArmNumJoints):
            qd[i] = np.radians(
                self.feedback.actuators[i].velocity
            )  # Kortex provides joint angles
            # in degrees for some reason
        output.SetFromVector(qd)

    def CalcArmTorque(self, context, output):
        t = context.get_time()
        if self.last_feedback_time != t:
            self.GetFeedback(t)

        tau = np.zeros(kGen3ArmNumJoints)
        for i in range(kGen3ArmNumJoints):
            tau[i] = np.radians(self.feedback.actuators[i].torque)  # in Nm

        output.SetFromVector(tau)

    def CalcState(self, context, output):
        t = context.get_time()
        if self.last_feedback_time != t:
            self.GetFeedback(t)

        q = np.zeros(kGen3ArmNumJoints)
        for i in range(kGen3ArmNumJoints):
            q[i] = np.radians(
                self.feedback.actuators[i].position
            )  # Kortex provides joint angles
        qd = np.zeros(kGen3ArmNumJoints)
        for i in range(kGen3ArmNumJoints):
            qd[i] = np.radians(
                self.feedback.actuators[i].velocity
            )  # Kortex provides joint angles
            # in degrees for some reason

        output.SetFromVector(np.concatenate((q, qd), axis=0))

    def CalcEndEffectorPose(self, context, output):
        """
        Compute the current end-effector pose and send as output.
        """
        t = context.get_time()
        if self.last_feedback_time != t:
            self.GetFeedback(t)

        ee_pose = np.zeros(6)
        ee_pose[0] = np.radians(self.feedback.base.tool_pose_theta_x)
        ee_pose[1] = np.radians(self.feedback.base.tool_pose_theta_y)
        ee_pose[2] = np.radians(self.feedback.base.tool_pose_theta_z)
        ee_pose[3] = self.feedback.base.tool_pose_x
        ee_pose[4] = self.feedback.base.tool_pose_y
        ee_pose[5] = self.feedback.base.tool_pose_z

        # pose : RigidTransform = self.X_WB @ RigidTransform(RollPitchYaw(ee_pose[:3]).ToQuaternion(), ee_pose[3:])
        # o = np.zeros(7)
        # # o[:4] = pose.rotation().ToQuaternion().wxyz()
        # o[:3] = np.array([0, 3.14, 0])
        # o[3:-1] = pose.translation()
        output.SetFromVector(ee_pose)
