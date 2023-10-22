
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