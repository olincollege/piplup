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

        self.Kp = Kp
        self.Kd = Kd

    def CalcGripperCommand(self, context, output):
        t = context.get_time()
        cmd_pos = np.array([1.0])  # TODO from gamepad
        output.SetFromVector(cmd_pos)

    def CalcEndEffectorCommand(self, context, output):
        t = context.get_time()

        target_twist = np.zeros(6) # TODO from gamepad

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

