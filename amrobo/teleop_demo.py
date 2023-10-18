
from pydrake.all import *
import numpy as np
import matplotlib.pyplot as plt
import pygame

from kinova_station import KinovaStation, EndEffectorTarget, GripperTarget
from controllers import GamepadTeleopController

station = KinovaStation(time_step=0.0005, n_dof=7)
station.AddGround()
station.AddArmWith2f85Gripper(arm_damping=False)
station.ConnectToDrakeVisualizer()
X = RigidTransform()
X.set_translation([0.5,0,0.1])
X.set_rotation(RotationMatrix(RollPitchYaw([-np.pi/2,0,0])))
station.AddManipulandFromFile("/home/ksuresh/drake/manipulation/models/ycb/sdf/003_cracker_box.sdf", X)
station.ConnectToMeshcatVisualizer()
station.Finalize()

builder = DiagramBuilder()
station = builder.AddSystem(station)


Kp = 10*np.eye(6)
Kd = 2*np.sqrt(Kp)

pygame.init()
pygame.joystick.init()
gamepad = pygame.joystick.Joystick(0)

controller = builder.AddSystem(GamepadTeleopController(
    gamepad, # TODO get from pygame
    command_type=EndEffectorTarget.kTwist,
    Kp=Kp,
    Kd=Kd))
controller.set_name("controller")
controller.ConnectToStation(builder, station)

# Build the system diagram
diagram = builder.Build()
diagram.set_name("system_diagram")
diagram_context = diagram.CreateDefaultContext()

station.go_home(diagram, diagram_context, name="Home")

# plt.figure()
# plot_system_graphviz(station,max_depth=1)
# plt.show()

station.SetManipulandStartPositions(diagram, diagram_context)

# Set up simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Run simulation
simulator.Initialize()
station.meshcat.AddButton("Stop Simulation", "Escape")
print("Press Escape to stop the simulation")
while station.meshcat.GetButtonClicks("Stop Simulation") < 1:
    simulator.AdvanceTo(simulator.get_context().get_time()+1.0)
station.meshcat.DeleteButton("Stop Simulation")
