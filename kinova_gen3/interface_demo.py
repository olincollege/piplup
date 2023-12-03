from kinova_gen3 import Gen3HardwareInterface, Gen3ControlMode
from pydrake.all import *
import numpy as np


class TestSys(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclarePeriodicUnrestrictedUpdateEvent(1 / 40, 0, self.Integrate)

    def Integrate(self, context: Context, discrete_state: DiscreteValues):
        # def DoCalcTimeDerivatives(self, context, continuous_state):
        print("time is: %s" % context.get_time())


if __name__ == "__main__":
    # sys = TestSys()
    sys = Gen3HardwareInterface("192.168.1.10", "10000", Gen3ControlMode.kTwist)
    sim = Simulator(sys)
    context = sim.get_context()
    # pose = sys.GetOutputPort("pose_measured").Eval(context)
    # test_pose = RigidTransform(RollPitchYaw(pose[:3]).ToQuaternion(),pose[3:])
    sys.GetInputPort("twist").FixValue(context, np.zeros(6))

    sim.set_target_realtime_rate(1.0)

    try:
        sim.AdvanceTo(math.inf)
    except KeyboardInterrupt:
        print(sim.get_actual_realtime_rate())
        sys.CleanUp()
