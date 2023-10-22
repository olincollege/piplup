from pydrake.all import *
import numpy as np
import matplotlib.pyplot as plt
from kinova_station import KinovaStation, EndEffectorTarget, GripperTarget
from controllers import GamepadController, DifferentialInverseKinematicsController, InverseKinematicsController

station = KinovaStation(time_step=0.0005)
station.AddGround()
station.ConnectToMeshcatVisualizer()
station.Finalize()

builder = DiagramBuilder()
station: KinovaStation = builder.AddSystem(station)

controller_plant = station.GetSubsystemByName(
    "gen3_controller"
).get_multibody_plant_for_control()
frame = controller_plant.GetFrameByName("end_effector_frame")

gamepad: GamepadController = builder.AddSystem(GamepadController(station.meshcat))
ik_controller: InverseKinematicsController = builder.AddSystem(
    InverseKinematicsController(controller_plant, station.meshcat)
)

builder.Connect(
    station.GetOutputPort("gen3.ee_pose"),
    gamepad.GetInputPort("ee_pose")
)

builder.Connect(
    gamepad.GetOutputPort("ee_command"),
    ik_controller.GetInputPort("V_WE_desired"),
)
builder.Connect(
    gamepad.GetOutputPort("gripper_command"),
    station.GetInputPort("gripper.target"),
)

builder.Connect(
    station.GetOutputPort("gen3.measured_position"),
    ik_controller.GetInputPort("joint_position"),
)
builder.Connect(
    station.GetOutputPort("gen3.measured_velocity"),
    ik_controller.GetInputPort("joint_velocity"),
)

builder.Connect(
    ik_controller.GetOutputPort("joint_position_target"),
    station.GetInputPort("gen3.joint"),
)
station.meshcat.SetObject("test", Sphere(0.05), Rgba(0,0.5,0,0.5))
diagram = builder.Build()
diagram.set_name("system_diagram")


diagram_context = diagram.CreateDefaultContext()

station.go_home(diagram, diagram_context, name="Home")

# plt.figure()
# plot_system_graphviz(diagram, max_depth=1)
# plt.show()


# Set up simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Run simulation
simulator.Initialize()
station.meshcat.AddButton("Stop Simulation", "Escape")
print("Press Escape to stop the simulation")
while station.meshcat.GetButtonClicks("Stop Simulation") < 1:
    simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
station.meshcat.DeleteButton("Stop Simulation")
