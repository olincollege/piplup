from pydrake.all import *
import numpy as np

class GamepadController(LeafSystem):
    """
    A controller that converts gamepad input to end effector commands.

                    -------------------------
                    |                       |
                    |                       |
                    |                       | ---> ee_command (velocity)
       ee_pose ---> |                       | 
                    |   GamepadController   |
                    |                       |
                    |                       | ---> gripper_command (velocity)
                    |                       | 
                    |                       |
                    -------------------------

    """
    def __init__(self, meshcat):
        super().__init__()

        self._meshcat = meshcat
        self.linear_speed = 0.5
        self.angular_speed = 3

        self.ee_pose_port = self.DeclareVectorInputPort(
                                    "ee_pose",
                                    BasicVector(6)) 

        self.DeclareVectorOutputPort(
            "ee_command",
            BasicVector(6),
            self.CalcEndEffectorCommand
        )

        self.DeclareVectorOutputPort(
            "gripper_command",
            BasicVector(1),
            self.CalcGripperCommand
        )

    def CalcEndEffectorCommand(self, context, output):
        current_pose = self.ee_pose_port.Eval(context)

        gamepad = self._meshcat.GetGamepad()

        def CreateStickDeadzone(x, y):
            stick = np.array([x, y])
            deadzone = 0.3
            m = np.linalg.norm(stick)
            if m < deadzone:
                return np.array([0, 0])
            over = (m - deadzone) / (1 - deadzone)
            return stick * over / m
        
        left = CreateStickDeadzone(gamepad.axes[0], -gamepad.axes[1])
        right = CreateStickDeadzone(-gamepad.axes[3], gamepad.axes[4])
        z_vel = gamepad.button_values[3] * (self.linear_speed/2) - gamepad.button_values[0] * (self.linear_speed/2)

        target_twist = np.zeros(6)

        target_twist[3:-1] = left * self.linear_speed
        target_twist[-1] = z_vel

        ee_rot = RotationMatrix(Quaternion(current_pose[:4]))
        
        target_twist[:3] = ee_rot.inverse().multiply(np.array([right[1], 0, right[0]]))

        output.SetFromVector(target_twist)
        

    def CalcGripperCommand(self, context, output):
        gamepad = self._meshcat.GetGamepad()

        gripper_close = gamepad.axes[2]
        gripper_open = gamepad.axes[5]
        cmd_pos = np.array([(gripper_close - gripper_open)/2]) 
        output.SetFromVector(cmd_pos)
    