import dataclasses as dc


@dc.dataclass
class Gen3InterfaceConfig:
    port: str
    ip_address: str


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
