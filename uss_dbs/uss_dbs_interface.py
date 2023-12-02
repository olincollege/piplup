from pydrake.all import *
import serial

from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace


# Not currently used
class USSDBSInterfaceConfig:
    __fields__: ClassVar[tuple] = (SimpleNamespace(name="port", type=str),)
    port: str


class USSDBSHardwareInterface(LeafSystem):
    def __init__(self, port="/dev/ttyUSB0"):
        LeafSystem.__init__(self)
        self.scale_serial = serial.Serial(port)
        self.mass_state_idx = self.DeclareDiscreteState(1)
        self.DeclareStateOutputPort("mass", self.mass_state_idx)
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.001, 0.0, self.ReadSerial)

    def ReadSerial(self, context: Context, state: State):
        self.scale_serial
        serial_data = self.scale_serial.readline().split()
        if len(serial_data) != 2:
            raise RuntimeError("Missing Serial Data")
        state.get_mutable_discrete_state(self.mass_state_idx)[0] = float(
            serial_data[0] + serial_data[1][:-1]
        )
