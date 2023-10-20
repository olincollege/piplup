
from pydrake.all import *
import numpy as np
import matplotlib.pyplot as plt

from kinova_station import KinovaStation

station = KinovaStation(time_step=0.0005)
station.AddGround()
station.ConnectToMeshcatVisualizer()
station.Finalize()

builder = DiagramBuilder()
station = builder.AddSystem(station)
diagram = builder.Build()
diagram.set_name("system_diagram")

# plt.figure()
# plot_system_graphviz(station,max_depth=1)
# plt.show()

diagram_context = diagram.CreateDefaultContext()

station.go_home(diagram, diagram_context, name="Home")

# plt.figure()
# plot_system_graphviz(station,max_depth=1)
# plt.show()

# station.SetManipulandStartPositions(diagram, diagram_context)

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
