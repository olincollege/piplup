from pydrake.all import *
from kinova_station import GripperTarget, JointTarget
import numpy as np


class DifferentialInverseKinematicsController(LeafSystem):
    """
                            -------------------------
                            |                       |
                            |                       |
                            |                       |
    V_WE_desired            |                       | ---> joint_command_velocity
    joint_position -------> |                       |
    joint_velocity -------> |                       |
                            |                       |
                            |                       |
                            -------------------------

    """

    def __init__(self, plant: MultibodyPlant, frame_E: Frame):
        LeafSystem.__init__(self)
        self.V_WE_desired_port = self.DeclareVectorInputPort(
            "V_WE_desired", BasicVector(6)
        )
        self.joint_position_port = self.DeclareVectorInputPort(
            "joint_position", BasicVector(7)
        )
        self.joint_velocity_port = self.DeclareVectorInputPort(
            "joint_velocity", BasicVector(7)
        )

        self.DeclareVectorOutputPort(
            "joint_command_velocity", BasicVector(7), self.CalcJointCommand
        )

        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self.GetJointLimits()

        if frame_E is None:
            frame_E = plant.GetFrameByName("end_effector_frame")

        self._frame_E = frame_E

        params = DifferentialInverseKinematicsParameters(
            plant.num_positions(), plant.num_velocities()
        )
        # q0 = plant.GetPositions(plant.CreateDefaultContext())
        # params.set_nominal_joint_position(q0)
        params.set_end_effector_angular_speed_limit(2)
        params.set_end_effector_translational_velocity_limits([-2, -2, -2], [2, 2, 2])
        # params.set_joint_centering_gain(10 * np.eye(7))
        params.set_joint_velocity_limits((self.qd_min, self.qd_max))
        params.set_joint_position_limits((self.q_min, self.q_max))

        self._diff_ik_params = params

    def GetJointLimits(self):
        q_min = []
        q_max = []
        qd_min = []
        qd_max = []

        joint_indices = self._plant.GetJointIndices(
            self._plant.GetModelInstanceByName("Kinova_Gen3")
        )

        for idx in joint_indices:
            joint = self._plant.get_joint(idx)

            if joint.type_name() == "revolute":  # ignore the joint welded to the world
                q_min.append(joint.position_lower_limit())
                q_max.append(joint.position_upper_limit())
                qd_min.append(joint.velocity_lower_limit())
                qd_max.append(joint.velocity_upper_limit())

        self.q_min = np.array(q_min)
        self.q_max = np.array(q_max)
        self.qd_min = np.array(qd_min)
        self.qd_max = np.array(qd_max)

    def CalcJointCommand(self, context, output):
        V_WE_desired = self.V_WE_desired_port.Eval(context)
        q = self.joint_position_port.Eval(context)
        qd = self.joint_velocity_port.Eval(context)
        self._plant.SetPositions(self._plant_context, q)
        self._plant.SetVelocities(self._plant_context, qd)
        result: DifferentialInverseKinematicsResult = DoDifferentialInverseKinematics(
            self._plant,
            self._plant_context,
            V_WE_desired,
            self._frame_E,
            self._diff_ik_params,
        )

        cmd = np.zeros(7)
        if result.status != DifferentialInverseKinematicsStatus.kNoSolutionFound:
            cmd = result.joint_velocities
        print(V_WE_desired)
        print(cmd)
        output.SetFromVector(cmd)
