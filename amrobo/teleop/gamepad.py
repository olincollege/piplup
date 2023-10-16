"""
Implementing Xbox controller bindings in pygame

Ref: https://www.pygame.org/docs/ref/joystick.html
"""
from enum import Enum

import pygame
import numpy as np
import time
from pydrake.all import (
    LeafSystem,
    Simulator,
)

class Mode(Enum):
    LINEAR = 0
    ANGULAR = 1

class Gamepad(LeafSystem):
    def __init__(self, joystick):
        super().__init__()

        self.joystick = joystick
        self.mode = Mode.LINEAR

        self.linear_speed = 0.5
        self.angular_speed = 0.5

        self.linear_vel = [0.0,0.0,0.0]
        self.angular_vel = [0.0,0.0,0.0]

        state = self.DeclareDiscreteState(6)

        self.DeclareStateOutputPort("vel", state)

        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.1,
            offset_sec=0.0,
            update=self.update
        )

    def update(self, context, state):
        pygame.event.get()

        # If X button pressed, switch modes
        if self.joystick.get_button(2):
            state.get_mutable_vector().SetFromVector([0.,0.,0.,0.,0.,0.])
            if self.mode == Mode.LINEAR:
                self.mode = Mode.ANGULAR
            else:
                self.mode = Mode.LINEAR

        # Set vels according to joystick inputs
        if self.mode == Mode.LINEAR:
            state.get_mutable_vector().SetAtIndex(1, -self.joystick.get_axis(0) * self.linear_speed)
            state.get_mutable_vector().SetAtIndex(0, -self.joystick.get_axis(1) * self.linear_speed)
            state.get_mutable_vector().SetAtIndex(2, -self.joystick.get_axis(4) * self.linear_speed)

            self.linear_speed += self.joystick.get_hat(0)[1] * 0.1

        elif self.mode == Mode.ANGULAR:
            state.get_mutable_vector().SetAtIndex(4, -self.joystick.get_axis(0) * self.angular_speed)
            state.get_mutable_vector().SetAtIndex(3, -self.joystick.get_axis(1) * self.angular_speed)
            state.get_mutable_vector().SetAtIndex(5, -self.joystick.get_axis(4) * self.angular_speed)

            self.angular_speed += self.joystick.get_hat(0)[1] * 0.1
        
        print(context.get_discrete_state_vector())

def main():
    # Instantiate the System.
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    print(joystick.get_hat(0))

    system = Gamepad(joystick)
    simulator = Simulator(system)
    context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.)
    
    context.get_mutable_discrete_state_vector().SetFromVector([0.,0.,0.,0.,0.,0.])
    simulator.AdvanceTo(60)


if __name__ == '__main__':
    main()