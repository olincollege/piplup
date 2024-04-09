from pydrake.all import *
from enum import Enum, auto

from kinova_gen3 import Gen3ControlMode, Gen3NamedPosition, kGen3NamedPositions
import time
from time import sleep

from common.logging import *

def calc_distance(mass, base_torque, loaded_torque):
    return (loaded_torque - base_torque) / (mass * 9.8)

class COMPlannerState(Enum):
    INIT = auto()
    MOVE_TO_NEUTRAL = auto()
    MOVE_TO_PRE_SCAN = auto()
    SCAN_MANIPULAND = auto()
    CALC_PICK_POSE = auto()
    MOVE_TO_PRE_PICK = auto()
    MOVE_TO_PICK = auto()
    CLOSE_GRIPPER = auto()
    PICK_MANIPULAND = auto()
    MOVE_TO_POSITION_1 = auto()
    MEASURE_AT_POSITION_1 = auto()
    MOVE_TO_POSITION_2 = auto()
    MEASURE_AT_POSITION_2 = auto()
    MOVE_TO_POSITION_3 = auto()
    MEASURE_AT_POSITION_3 = auto()
    DROP_MANIPULAND = auto()
    IDLE = auto()


class COMPlanner(LeafSystem):
    def __init__(self, meshcat=None, mass=.275):
        LeafSystem.__init__(self)

        self.DeclareAbstractInputPort(
            "merged_point_cloud", AbstractValue.Make(PointCloud(0))
        )

        # States
        self.planner_state_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(COMPlannerState.INIT)
        )
        self.control_mode_idx_ = self.DeclareAbstractState(
            AbstractValue.Make(Gen3ControlMode.kTwist)
        )
        self.command_idx_ = self.DeclareDiscreteState(7)
        # TODO: What is neutral and what is closed? 0-1?
        self.gripper_command_idx_ = self.DeclareDiscreteState(1)

        # Input Ports
        self.planar_grasp_port_ = self.DeclareAbstractInputPort(
            "planar_grasp_selection",
            AbstractValue.Make((np.inf, np.inf, RigidTransform(), np.inf)),
        )

        self.meshcat = meshcat

        self.arm_torque_port_ = self.DeclareVectorInputPort("torque_measured", 7)

        self.calibration_torques = [0, 0, 0]
        self.calibration = True

        self.num_measurements = 100

        #temp solution for demo - shouldn't be variable passed in while initialized
        #obejct mass in kg
        self.mass = mass

        # Output Ports
        self.DeclareStateOutputPort("arm_command", self.command_idx_)
        self.DeclareStateOutputPort("control_mode", self.control_mode_idx_)
        self.DeclareStateOutputPort("gripper_command", self.gripper_command_idx_)

        self.DeclarePeriodicUnrestrictedUpdateEvent(0.0001, 0.0, self.Update)

    def change_planner_state(self, state: State, new_state: COMPlannerState):
        state.get_mutable_abstract_state(self.planner_state_idx_).set_value(new_state)

    def change_command_mode(self, state: State, new_mode: Gen3ControlMode):
        state.get_mutable_abstract_state(self.control_mode_idx_).set_value(new_mode)

    def Update(self, context: Context, state: State):
        planner_state = context.get_abstract_state(self.planner_state_idx_).get_value()
        logging.info(f"Current State: {planner_state}")

        match planner_state:
            case COMPlannerState.INIT:
                self.change_planner_state(state, COMPlannerState.MOVE_TO_NEUTRAL)
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([0.0])
                )
            case COMPlannerState.MOVE_TO_NEUTRAL:
                sleep(2)
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    kGen3NamedPositions[Gen3NamedPosition.NEUTRAL]
                )
                self.change_planner_state(state, COMPlannerState.MOVE_TO_PRE_SCAN)

            case COMPlannerState.MOVE_TO_PRE_SCAN:
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    kGen3NamedPositions[Gen3NamedPosition.CAMCLEAR]
                )
                if self.calibration:
                    self.change_planner_state(state, COMPlannerState.MOVE_TO_POSITION_1)
                else:
                    self.change_planner_state(state, COMPlannerState.SCAN_MANIPULAND)

            case COMPlannerState.SCAN_MANIPULAND:
                self.change_planner_state(state, COMPlannerState.CALC_PICK_POSE)

            case COMPlannerState.CALC_PICK_POSE:
                self.planar_grasp_selection = self.planar_grasp_port_.Eval(context)
                self.pick_pose_transform: RigidTransform = self.planar_grasp_selection[2] @ RigidTransform([0.0, 0.0, 0.1325])
                self.grip_width = self.planar_grasp_selection[3]
                self.change_planner_state(state, COMPlannerState.MOVE_TO_PRE_PICK)

            case COMPlannerState.MOVE_TO_PRE_PICK:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                rotation_matrix: RotationMatrix = self.pick_pose_transform.rotation() @ RollPitchYaw([0, 0, -np.pi/2]).ToRotationMatrix()
                translation: np.ndarray = self.pick_pose_transform.translation()
                self.pick_pose_array = np.concatenate(
                    [rotation_matrix.ToRollPitchYaw().vector(), translation, [0]]
                )
                logging.info(f"Pick pose: {self.pick_pose_array}")
                # Move along -z axis of gripper by 5cm
                pre_pick_offset = rotation_matrix.col(2) * -0.05
                pre_pick_pose = copy.copy(self.pick_pose_array)
                pre_pick_pose[3:6] += pre_pick_offset

                logging.info(f"Pre-pick pose: {pre_pick_pose}")

                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    pre_pick_pose
                )

                self.change_planner_state(state, COMPlannerState.MOVE_TO_PICK)

            case COMPlannerState.MOVE_TO_PICK:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                logging.info(f"Pick pose: {self.pick_pose_array}")
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    self.pick_pose_array
                )
                self.change_planner_state(state, COMPlannerState.CLOSE_GRIPPER)

            case COMPlannerState.CLOSE_GRIPPER:
                gripper_command = np.clip(1 - self.grip_width / 0.085 + 0.2, 0, 1)
                logging.info(f"Gripper command: {gripper_command}")
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([gripper_command])
                )
                self.change_planner_state(state, COMPlannerState.PICK_MANIPULAND)
                self.last_time = context.get_time()

            case COMPlannerState.PICK_MANIPULAND:
                if (context.get_time() - self.last_time) > 0.0005:
                    self.change_command_mode(state, Gen3ControlMode.kPose)
                    pick_up_pose = copy.copy(self.pick_pose_array)
                    pick_up_pose[5] += 0.1
                    state.get_mutable_discrete_state(self.command_idx_).set_value(
                        pick_up_pose
                    )
                    self.change_planner_state(state, COMPlannerState.MOVE_TO_POSITION_1)

            case COMPlannerState.MOVE_TO_POSITION_1:
                self.change_planner_state(state, COMPlannerState.DROP_MANIPULAND)
                self.change_command_mode(state, Gen3ControlMode.kPosition)
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    [0, 0, 0, 1.57, 0, 0, 0]
                )
                
                self.change_planner_state(state, COMPlannerState.MEASURE_AT_POSITION_1)

            case COMPlannerState.MEASURE_AT_POSITION_1:
                pick_torque = 0
                for i in range(self.num_measurements):
                        print(i)
                        pick_torque += self.arm_torque_port_.Eval(context)[3]
                        sleep(.1)
                pick_torque = pick_torque / self.num_measurements
                if self.calibration:
                    self.calibration_torques[0] = pick_torque
                else:
                    distance = calc_distance(self.mass, self.calibration_torques[0], pick_torque)
                    self.add_plane_to_meshcat(distance, "pos_1_plane")
                    print("Pos 1 dis: ", distance)

                print("POS 1: ", pick_torque)
                
                self.change_planner_state(state, COMPlannerState.MOVE_TO_POSITION_2)

            case COMPlannerState.MOVE_TO_POSITION_2:
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    [0, 0, 0, 1.57, 0, 1.57, 0]
                )
                self.change_planner_state(state, COMPlannerState.MEASURE_AT_POSITION_2)

            case COMPlannerState.MEASURE_AT_POSITION_2:
                pick_torque = 0
                for i in range(self.num_measurements):
                        pick_torque += self.arm_torque_port_.Eval(context)[3]
                        sleep(.1)
                pick_torque = pick_torque / self.num_measurements
                if self.calibration:
                    self.calibration_torques[1] = pick_torque
                else:
                    distance = calc_distance(self.mass, self.calibration_torques[1], pick_torque)
                    self.add_plane_to_meshcat(distance, "pos_2_plane")
                    print("Pos 2 dis: ", distance)
                    
                print("POS 2: ", pick_torque)
                self.change_planner_state(state, COMPlannerState.MOVE_TO_POSITION_3)

            case COMPlannerState.MOVE_TO_POSITION_3:
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    [0, 0, 0, 1.57, 1.57, 1.57, 0]
                )
                self.change_planner_state(state, COMPlannerState.MEASURE_AT_POSITION_3)

            case COMPlannerState.MEASURE_AT_POSITION_3:
                pick_torque = 0
                for i in range(self.num_measurements):
                        pick_torque += self.arm_torque_port_.Eval(context)[3]
                        sleep(.1)
                pick_torque = pick_torque / self.num_measurements
                if self.calibration:
                    self.calibration_torques[2] = pick_torque
                    self.calibration = False
                    self.change_planner_state(state, COMPlannerState.MOVE_TO_PRE_SCAN)
                    print("calibration: ", self.calibration_torques)
                else:
                    distance = calc_distance(self.mass, self.calibration_torques[2], pick_torque)
                    print("Pos 3 dis: ", distance)
                    self.add_plane_to_meshcat(distance, "pos_3_plane")
                    self.change_planner_state(state, COMPlannerState.DROP_MANIPULAND)
                print("POS 3: ", pick_torque)

            case COMPlannerState.DROP_MANIPULAND:
                self.change_command_mode(state, Gen3ControlMode.kPose)
                logging.info(f"Pick pose: {self.pick_pose_array}")
                state.get_mutable_discrete_state(self.command_idx_).set_value(
                    self.pick_pose_array
                )
                state.get_mutable_discrete_state(self.gripper_command_idx_).set_value(
                    np.array([0.0])
                )
                self.change_planner_state(state, COMPlannerState.IDLE)
                self.last_time = context.get_time()
            case COMPlannerState.IDLE:
                if (context.get_time() - self.last_time) > 0.001:
                    self.change_command_mode(state, Gen3ControlMode.kPosition)
                    state.get_mutable_discrete_state(self.command_idx_).set_value(
                        kGen3NamedPositions[Gen3NamedPosition.NEUTRAL]
                    )
            case _:
                logging.error("Invalid Planner State")

    
    def add_plane_to_meshcat(self, x_pos=.5, name="visualization_plane", plane_size=(1.0, 1.0, .01), color=Rgba(1, 0.5, 0.5, 0.5)):
        """
        Adds a visual representation of a plane to the Meshcat visualizer.

        Args:
        - z_position: The Z position of the plane in the world frame.
        - plane_size: The size of the plane as a tuple (length, width, height).
        - color: The color of the plane as an Rgba object.
        """
        # Define the plane geometry.
        plane_geometry = Box(plane_size[0], plane_size[1], plane_size[2])
        
        # Create a RigidTransform for the plane's pose. This example places the plane parallel to the X-Y plane.
        plane_pose = RigidTransform(RollPitchYaw(0, 1.57, 0), [x_pos, 0, 0])
        
        # Add the plane to Meshcat.
        self.meshcat.SetObject(f"/{name}", plane_geometry, color)
        self.meshcat.SetTransform(f"/{name}", plane_pose)

    # def CalcGen3Command(self, context: Context, output: BasicVector):
    #     cmd = context.get_discrete_state(self.command_idx_)
    #     output.SetFromVector(cmd)

