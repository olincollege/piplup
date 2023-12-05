from pydrake.all import *
from uss_dbs import USSDBSHardwareInterface

if __name__ == "__main__":
    scale_system = USSDBSHardwareInterface()
    simulator = Simulator(scale_system)

    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(5.0)
