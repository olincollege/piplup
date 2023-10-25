from pydrake.all import *
import numpy as np
import matplotlib.pyplot as plt
from kinova_station import KinovaStation, EndEffectorTarget, GripperTarget
from controllers import GamepadController, DifferentialInverseKinematicsController, InverseKinematicsController
class ToPose(LeafSystem):
    def __init__(self, grab_focus=True):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("rpy_xyz", 6)
        self.DeclareAbstractOutputPort(
            "pose", lambda: Value(RigidTransform()),
            self.DoCalcOutput)

    def DoCalcOutput(self, context, output):
        rpy_xyz = self.get_input_port().Eval(context)
        output.set_value(RigidTransform(RollPitchYaw(rpy_xyz[:3]),
                                        rpy_xyz[3:]))

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

params = DifferentialInverseKinematicsParameters(
    controller_plant.num_positions(), controller_plant.num_velocities()
)
# q0 = plant.GetPositions(plant.CreateDefaultContext())
# params.set_nominal_joint_position(q0)
time_step = 0.005
params.set_time_step(time_step)
params.set_end_effector_angular_speed_limit(2)
params.set_end_effector_translational_velocity_limits([-2, -2, -2], [2, 2, 2])
# params.set_joint_centering_gain(1 * np.eye(7))
params.set_joint_velocity_limits((station.gen3_controller.qd_min, station.gen3_controller.qd_max))
params.set_joint_position_limits((station.gen3_controller.q_min, station.gen3_controller.q_max))

differential_ik : DifferentialInverseKinematicsIntegrator = builder.AddSystem(
    DifferentialInverseKinematicsIntegrator(
        controller_plant, frame, time_step, params))

filter : FirstOrderLowPassFilter = builder.AddSystem(
    FirstOrderLowPassFilter(time_constant=0.1, size=6))

builder.Connect(gamepad.get_output_port(0), filter.get_input_port(0))

to_pose : ToPose = builder.AddSystem(ToPose())
builder.Connect(filter.get_output_port(0),
                to_pose.get_input_port())

builder.Connect(to_pose.get_output_port(),
                differential_ik.GetInputPort("X_WE_desired"))

builder.Connect(,
                differential_ik.GetInputPort("robot_state"))
builder.Connect(
    to_pose.get_output_port(),
    gamepad.GetInputPort("ee_pose")
)

builder.Connect(
    gamepad.GetOutputPort("gripper_command"),
    station.GetInputPort("gripper.target"),
)

builder.Connect(
    differential_ik.GetOutputPort("joint_positions"),
    station.GetInputPort("gen3.joint"),
)

station.meshcat.SetObject("test", Sphere(0.05), Rgba(0,0.5,0,0.5))
ee_base = Mesh("../models/robotiq_description/meshes/visual/robotiq_arg2f_85_base_link.obj",1)
station.meshcat.SetObject("target", ee_base, Rgba(0,0.5,0,0.5))
station.meshcat.SetCameraPose(np.array([1,-1,1])*0.75, np.array([0,0,0.4]))


diagram : Diagram = builder.Build()
diagram.set_name("system_diagram")

diagram_context = diagram.CreateDefaultContext()

station.go_home(diagram, diagram_context, name="Home")

plt.figure()
plot_system_graphviz(diagram, max_depth=1)
plt.show()


# Set up simulation
simulator = Simulator(diagram, diagram_context)
station_context = diagram.GetMutableSubsystemContext(
    station, simulator.get_mutable_context())

q0 = station.GetOutputPort("gen3.measured_position").Eval(
    station_context)
differential_ik.get_mutable_parameters().set_nominal_joint_position(q0)

differential_ik.SetPositions(
    differential_ik.GetMyMutableContextFromRoot(
        simulator.get_mutable_context()), q0)

ctx = differential_ik.GetMyMutableContextFromRoot(simulator.get_mutable_context())
ctx.FixInputPort(differential_ik.GetInputPort("use_robot_state"), Value(True))

simulator.set_target_realtime_rate(1.0)
# Run simulation
simulator.Initialize()
station.meshcat.AddButton("Stop Simulation", "Escape")
print("Press Escape to stop the simulation")
while station.meshcat.GetButtonClicks("Stop Simulation") < 1:
    simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
station.meshcat.DeleteButton("Stop Simulation")
