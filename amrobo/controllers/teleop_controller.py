import pygame
from kinova_station import EndEffectorTarget
from controllers.basic_controller import *
from controllers.command_sequence import *



class GamepadTeleopController(BasicController):
    def __init__(self, gamepad, 
                       command_type = EndEffectorTarget.kTwist, 
                       Kp = 10*np.eye(6),
                       Kd = 2*np.sqrt(10)*np.eye(6)):

        BasicController.__init__(self, command_type=command_type)

        self.gamepad = gamepad
        self.mode = 1
        self.linear_speed = 0.5
        self.angular_speed = 5
        self.deadzone = 0.1

        self.Kp = Kp
        self.Kd = Kd
        self.target_pose = None

    def SetGripperCommandType(self, context, output):
        command_type = GripperTarget.kVelocity
        output.SetFrom(AbstractValue.Make(command_type))

    def CalcGripperCommand(self, context, output):
        t = context.get_time()
        pygame.event.get()
        gripper_close = self.gamepad.get_axis(2)
        gripper_open = self.gamepad.get_axis(5)
        cmd_pos = np.array([(gripper_close - gripper_open)/2]) 
        output.SetFromVector(cmd_pos)

    def CalcEndEffectorCommand(self, context, output):
        t = context.get_time()
        current_pose = self.ee_pose_port.Eval(context)
        current_twist = self.ee_twist_port.Eval(context)

        if self.target_pose is None:
            self.prev_t = t
            self.target_pose = current_pose

        time_delta = t-self.prev_t
        self.prev_t = t
        pygame.event.get()

        # If X button pressed, switch modes
        if self.gamepad.get_button(2):
            self.mode = 1
        if self.gamepad.get_button(1):
            self.mode = 0

        def CreateStickDeadzone(x, y):
            stick = np.array([x, y])
            deadzone = 0.3
            m = np.linalg.norm(stick)
            if m < deadzone:
                return np.array([0, 0])
            over = (m - deadzone) / (1 - deadzone)
            return stick * over / m
        left = CreateStickDeadzone(-self.gamepad.get_axis(1), -self.gamepad.get_axis(0))
        right = CreateStickDeadzone(-self.gamepad.get_axis(4), -self.gamepad.get_axis(3))
        control_input = [left[0],left[1],right[0]]

        if self.mode == 0:
            self.target_pose[3:] += np.array(control_input) * self.linear_speed * time_delta 
            self.linear_speed += self.gamepad.get_hat(0)[1] * 0.1

        elif self.mode == 1:
            R_current = RotationMatrix(RollPitchYaw(self.target_pose[:3]))
            R_delta = RotationMatrix(RollPitchYaw(np.array(control_input) * self.angular_speed * time_delta))
            self.target_pose[:3] = RollPitchYaw(R_current.multiply(R_delta)).vector()
            # print()
            # print(np.array(control_input) * self.angular_speed * time_delta)
            # print(self.target_pose[:3])
            self.angular_speed += self.gamepad.get_hat(0)[1] * 0.1

        
        target_twist = np.zeros(6)
        twist_err = target_twist - current_twist
        pose_err = self.target_pose - current_pose

        R_current = RotationMatrix(RollPitchYaw(current_pose[:3]))
        R_target = RotationMatrix(RollPitchYaw(self.target_pose[:3]))
        R_err = R_target.multiply(R_current.transpose())
        pose_err[:3] = RollPitchYaw(R_err).vector()

        cmd = self.Kp@pose_err + self.Kd@twist_err

        output.SetFromVector(cmd)

    def ConnectToStation(self, builder, station):
        """
        Connect inputs and outputs of this controller to the given kinova station (either
        hardware or simulation). 
        """
        builder.Connect(                                  # Send commands to the station
                self.GetOutputPort("ee_command"),
                station.GetInputPort("ee_target"))
        builder.Connect(
                self.GetOutputPort("ee_command_type"),
                station.GetInputPort("ee_target_type"))
        builder.Connect(
                self.GetOutputPort("gripper_command"),
                station.GetInputPort("gripper_target"))
        builder.Connect(
                self.GetOutputPort("gripper_command_type"),
                station.GetInputPort("gripper_target_type"))

        builder.Connect(                                     # Send state information
                station.GetOutputPort("measured_ee_pose"),   # to the controller
                self.GetInputPort("ee_pose"))
        builder.Connect(
                station.GetOutputPort("measured_ee_twist"),
                self.GetInputPort("ee_twist"))

