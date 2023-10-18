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
        self.mode = 0
        self.linear_speed = 3
        self.angular_speed = 3
        self.deadzone = 0.05

        self.Kp = Kp
        self.Kd = Kd

    def CalcGripperCommand(self, context, output):
        t = context.get_time()
        pygame.event.get()
        gripper_close = self.gamepad.get_axis(2)
        gripper_open = self.gamepad.get_axis(5)
        cmd_pos = np.array([(gripper_close - gripper_open)/2 + 0.5])  # TODO from gamepad
        output.SetFromVector(cmd_pos)

    def CalcEndEffectorCommand(self, context, output):
        t = context.get_time()

        target_twist = np.zeros(6)

        pygame.event.get()

        # If X button pressed, switch modes
        if self.gamepad.get_button(2):
            target_twist = np.zeros(6)
            if self.mode == 0:
                self.mode = 1
            else:
                self.mode = 0

        # Set vels according to joystick inputs
        control_input = [-self.gamepad.get_axis(1), -self.gamepad.get_axis(0), -self.gamepad.get_axis(4)]
        if np.linalg.norm(control_input) < self.deadzone:
            control_input = np.zeros(3)
        if self.mode == 0:
            target_twist[3] = control_input[0] * self.linear_speed
            target_twist[4] = control_input[1] * self.linear_speed
            target_twist[5] = control_input[2] * self.linear_speed

            self.linear_speed += self.gamepad.get_hat(0)[1] * 0.1

        elif self.mode == 1:
            target_twist[0] = control_input[0] * self.angular_speed
            target_twist[1] = control_input[1] * self.angular_speed
            target_twist[2] = control_input[2] * self.angular_speed

            self.angular_speed += self.gamepad.get_hat(0)[1] * 0.1

        print(target_twist)

        current_pose = self.ee_pose_port.Eval(context)
        current_twist = self.ee_twist_port.Eval(context)

        # Compute pose and twist errors
        twist_err = target_twist - current_twist

        cmd = self.Kd@twist_err

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

