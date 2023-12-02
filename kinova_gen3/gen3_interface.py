import dataclasses as dc

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
from .gen3_constants import Gen3ControlMode, kGen3ArmNumJoints


class Gen3InterfaceConfig:
    __fields__: ClassVar[tuple] = (
        SimpleNamespace(name="port", type=str),
        SimpleNamespace(name="ip_address", type=str),
    )
    port: str
    ip_address: str


class Gen3HardwareInterface(LeafSystem):
    def __init__(self, ip_address, port, control_mode):
        LeafSystem.__init__(self)
        if control_mode == Gen3ControlMode.kPosition:
            self.arm_position_input_port = self.DeclareVectorInputPort(
                "position", kGen3ArmNumJoints
            )
        elif control_mode == Gen3ControlMode.kPose:
            self.arm_position_input_port = self.DeclareAbstractInputPort(
                "pose", Value(RigidTransform())
            )
        else:
            raise RuntimeError(f"Unsupported control mode {control_mode}")
        self.gripper_position_input_port = self.DeclareVectorInputPort(
            "2f_85.command", 1
        )
        self.DeclareVectorOutputPort(
            "position_measured", BasicVector(kGen3ArmNumJoints), self.CalcArmPosition
        )
        self.DeclareVectorOutputPort(
            "velocity_measured", BasicVector(kGen3ArmNumJoints), self.CalcArmVelocity
        )
        self.DeclareVectorOutputPort(
            "torque_measured", BasicVector(kGen3ArmNumJoints), self.CalcArmTorque
        )
        self.DeclareVectorOutputPort(
            "state_estimated", BasicVector(kGen3ArmNumJoints * 2), self.CalcState
        )
        self.DeclareVectorOutputPort(
            "pose_measured",
            BasicVector(6),
            self.CalcEndEffectorPose,
            {self.time_ticket()},
        )
        self.DeclareContinuousState(1)

        self.last_feedback_time = -np.inf
        self.feedback = None

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
            print(self.base.GetArmState())
            raise RuntimeError(
                "Arm not in ready state. Clear any faults before trying again."
            )

    def CleanUp(self):
        print("\nClosing Hardware Connection...")

        if self.session_manager is not None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000

            self.session_manager.CloseSession()

        self.transport.disconnect()
        print("Hardware Connection Closed.")

    def GetFeedback(self, current_time):
        """
        Sets self.feedback with the latest data from the controller.
        We also indicate the time at which this feedback was set, in an
        effor to reduce redudnant calls to self.base_cyclic.RefreshFeedback(),
        which take about 25ms each.
        """
        self.feedback = self.base_cyclic.RefreshFeedback()
        self.last_feedback_time = current_time

    def DoCalcTimeDerivatives(self, context, continuous_state):
        # print("time is: %s" % context.get_time())
        pass

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

        # Store the end-effector pose so we can use it to compute the camera pose
        self.ee_pose = ee_pose

        output.SetFromVector(ee_pose)
