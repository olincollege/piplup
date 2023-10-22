
from pydrake.all import *
import numpy as np


class InverseKinematicsController(LeafSystem):
    """
                            -------------------------
                            |                       |
                            |                       |
                            |                       |
    V_WE_desired   -------> |                       | 
    joint_position -------> |                       | ---> joint_position_target
    joint_velocity -------> |                       |
                            |                       |
                            |                       |
                            -------------------------

    """

    def __init__(self, plant: MultibodyPlant, meshcat):
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
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)
        self.DeclareVectorOutputPort(
            "joint_position_target", BasicVector(7), self.CalcJointCommand
        )
        self.meshcat = meshcat
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()

        self.DeclareDiscreteState(7)
        self._time_step = 0.005
        self.DeclarePeriodicDiscreteUpdateEvent(
            self._time_step, 0, self.Integrate
        )

        self.world_frame = self._plant.world_frame()
        self.ee_frame = self._plant.GetFrameByName("end_effector_frame")

    def Initialize(self, context, discrete_state):
        q = self.joint_position_port.Eval(context)
        self._plant.SetPositions(self._plant_context, q)
        ee_pose : RigidTransform = self._plant.CalcRelativeTransform(self._plant_context, self.world_frame, self.ee_frame)
        discrete_state.set_value(0, np.hstack((ee_pose.rotation().ToQuaternion().wxyz(), ee_pose.translation())))

    def Integrate(self, context, discrete_state):
        V_WE_desired = self.V_WE_desired_port.Eval(context)
        X_quat_trans = np.copy(context.get_discrete_state(0).get_value())
        X_WE_desired = RigidTransform(Quaternion(X_quat_trans[:4]), X_quat_trans[4:])

        q = self.joint_position_port.Eval(context)
        self._plant.SetPositions(self._plant_context, q)

        X_WE_desired.set_translation(X_WE_desired.translation() + V_WE_desired[3:] * self._time_step)

        R_delta = RotationMatrix(RollPitchYaw(V_WE_desired[:3] *  self._time_step))
        X_WE_desired.set_rotation(X_WE_desired.rotation().multiply(R_delta).ToQuaternion())
        discrete_state.set_value(0, np.hstack((X_WE_desired.rotation().ToQuaternion().wxyz(), X_WE_desired.translation())))
        self.meshcat.SetTransform("/drake/test", X_WE_desired)
        # print(context.get_discrete_state(0).get_value())

    def CalcJointCommand(self, context, output):
        q = self.joint_position_port.Eval(context)
        self._plant.SetPositions(self._plant_context, q)

        X_quat_trans = np.copy(context.get_discrete_state(0).get_value())
        X_WE_desired = RigidTransform(Quaternion(X_quat_trans[:4]), X_quat_trans[4:])

        ik = InverseKinematics(self._plant, self._plant_context)
        ik.AddPositionConstraint(self.ee_frame,
                                    [0,0,0],
                                    self.world_frame,
                                    X_WE_desired.translation(), 
                                    X_WE_desired.translation())
        ik.AddOrientationConstraint(self.ee_frame,
                                    RotationMatrix(),
                                    self.world_frame,
                                    X_WE_desired.rotation(),
                                    0.001)

        prog = ik.get_mutable_prog()
        q_var = ik.q()
        prog.AddQuadraticErrorCost(np.eye(len(q_var)), q, q_var)
        prog.SetInitialGuess(q_var, q)
        result = Solve(ik.prog())

        if not result.is_success():
            print("Inverse Kinematics Failed!")
            q_nom = np.zeros(self._plant.num_positions())
        else:
            q_nom = result.GetSolution(q_var)

        output.SetFromVector(q_nom)