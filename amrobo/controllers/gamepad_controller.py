from pydrake.all import *
import numpy as np

class GamepadController(LeafSystem):
    """
    A controller that converts gamepad input to end effector commands.

                    -------------------------
                    |                       |
                    |                       |
                    |                       | ---> X_WE_desired
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
        self.linear_speed = 0.1
        self.angular_speed = 0.8

        self.ee_pose_port : InputPort = self.DeclareAbstractInputPort(
                                    "ee_pose",
                AbstractValue.Make(RigidTransform()))

        self.DeclareAbstractOutputPort(
            "X_WE_desired",
            AbstractValue.Make(RigidTransform()),
            self.CalcEndEffectorCommand
        )

        self.DeclareVectorOutputPort(
            "gripper_command",
            BasicVector(1),
            self.CalcGripperCommand
        )
        self.linear_mode = True

    def CalcEndEffectorCommand(self, context, output):
        current_pose : RigidTransform = self.ee_pose_port.Eval(context)

        gamepad = self._meshcat.GetGamepad()
        if gamepad.index == None:
            target_twist = np.zeros(6)
            # target_twist[-1] = 0.05
            output.SetFromVector(target_twist)
            return 
        def CreateStickDeadzone(x, y):
            stick = np.array([x, y])
            deadzone = 0.3
            m = np.linalg.norm(stick)
            if m < deadzone:
                return np.array([0, 0])
            over = (m - deadzone) / (1 - deadzone)
            return stick * over / m
        
        left = CreateStickDeadzone(gamepad.axes[0], -gamepad.axes[1])
        right = CreateStickDeadzone(-gamepad.axes[2], gamepad.axes[3])

        if gamepad.button_values[8]:
            self.linear_mode = True
        if gamepad.button_values[9]:
            self.linear_mode = False

        target_twist = np.zeros(6)

        if self.linear_mode:
            target_twist[3:] = np.array([left[0], left[1], right[1]]) * self.linear_speed
        else:
            ee_rot = current_pose.rotation()
            target_twist[:3] = ee_rot.multiply(np.array([-left[0], right[1], left[1]]) * self.angular_speed)

        output.SetFromVector(target_twist)
        

    def CalcGripperCommand(self, context, output):
        gamepad = self._meshcat.GetGamepad()

        if gamepad.index == None:
            output.SetFromVector(np.zeros(1))
            return 
        gripper_close = gamepad.button_values[6]*3
        gripper_open = gamepad.button_values[7]*3
        cmd_pos = np.array([(gripper_close - gripper_open)/2]) 
        output.SetFromVector(cmd_pos)
    