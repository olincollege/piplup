from typing import Any, ClassVar, List, Optional
import sys
from types import SimpleNamespace
from pydrake.all import *
from .gen3_constants import *
import numpy as np
from .gen3_sim_driver import SimGen3Driver


# from kortex_api.TCPTransport import TCPTransport
# from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
# from kortex_api.SessionManager import SessionManager

# from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
# from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
# from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient

# from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
# from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

# from kortex_api.autogen.messages import DeviceConfig_pb2, Session_pb2, Base_pb2, VisionConfig_pb2

# class Gen3HardwareInterface(LeafSystem):
#     def __init__(self, ip_address, port):
#         LeafSystem.__init__(self)

#         self.position_input_port = self.DeclareVectorInputPort("position", 7)
#         self.position_input_port = self.DeclareVectorInputPort("position", 7)

#         self.DeclareContinuousState(1)

#         self.last_feedback_time = -np.inf
#         self.feedback = None

#         self.transport = TCPTransport()
#         self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
#         self.transport.connect(ip_address, port)

#         # Create session
#         session_info = Session_pb2.CreateSessionInfo()
#         session_info.username = "admin"
#         session_info.password = "admin"
#         session_info.session_inactivity_timeout = 60000   # (milliseconds)
#         session_info.connection_inactivity_timeout = 2000 # (milliseconds)

#         self.session_manager = SessionManager(self.router)
#         self.session_manager.CreateSession(session_info)

#         # Create required services
#         device_config = DeviceConfigClient(self.router)
#         self.base = BaseClient(self.router)
#         self.base_cyclic = BaseCyclicClient(self.router)

#         if self.base.GetArmState().active_state != Base_pb2.ARMSTATE_SERVOING_READY:
#             # TODO replace with logging
#             print("")
#             print("ERROR: arm not in ready state.")

#             print(self.base.GetArmState())

#             print("Make sure there is nothing else currently sending commands (e.g. joystick, web interface), ")
#             print("and clear any faults before trying again.")
#             sys.exit(0)


def AddSimGen3Driver(
    plant: MultibodyPlant,
    gen3_instance: ModelInstanceIndex,
    controller_plant: MultibodyPlant,
    builder: DiagramBuilder,
) -> System:
    inner_name = f"Gen3Driver({plant.GetModelInstanceName(gen3_instance)})"
    system: SimGen3Driver = builder.AddNamedSystem(
        inner_name, SimGen3Driver(controller_plant)
    )
    builder.Connect(
        plant.get_state_output_port(gen3_instance), system.GetInputPort("state")
    )
    builder.Connect(
        plant.get_generalized_contact_forces_output_port(gen3_instance),
        system.GetInputPort("generalized_contact_forces"),
    )
    builder.Connect(
        system.GetOutputPort("actuation"), plant.get_actuation_input_port(gen3_instance)
    )
    return system
