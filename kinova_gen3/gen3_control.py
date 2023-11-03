from typing import Any, ClassVar, List, Optional
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


# Build Controls
def BuildGen3Control():
    pass


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
