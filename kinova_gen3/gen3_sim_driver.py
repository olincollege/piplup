from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from .gen3_constants import *
import numpy as np

# Sim Gen3 Driver
class SimGen3Driver(Diagram):
    def __init__(self, controller_plant: MultibodyPlant):
        Diagram.__init__(self)
        builder = DiagramBuilder()
        num_positions = controller_plant.num_positions()
        state_demux: Demultiplexer = builder.AddNamedSystem(
            "demultiplexer", Demultiplexer(2 * num_positions, num_positions)
        )
        builder.ExportInput(state_demux.get_input_port(), "state")
        contact_forces: FirstOrderLowPassFilter = builder.AddNamedSystem(
            "low_pass_filter", FirstOrderLowPassFilter(0.01, num_positions)
        )
        builder.ExportInput(
            contact_forces.get_input_port(), "generalized_contact_forces"
        )

        inverse_dynamics: InverseDynamicsController = builder.AddNamedSystem(
            "inverse_dynamics_controller",
            InverseDynamicsController(
                controller_plant,
                kp=[100] * num_positions,
                ki=[1] * num_positions,
                kd=[20] * num_positions,
                has_reference_acceleration=False,
            ),
        )
        interpolator: StateInterpolatorWithDiscreteDerivative = builder.AddNamedSystem(
            "velocity_interpolator",
            StateInterpolatorWithDiscreteDerivative(
                num_positions,
                0.0005,
                suppress_initial_transient=True,
            ),
        )
        builder.ExportInput(interpolator.get_input_port(), "position")
        builder.ConnectInput("state", inverse_dynamics.GetInputPort("estimated_state"))
        builder.Connect(
            interpolator.GetOutputPort("state"),
            inverse_dynamics.GetInputPort("desired_state"),
        )

        builder.ExportOutput(inverse_dynamics.get_output_port(), "actuation")
        pos_pass = builder.AddNamedSystem(
            "position_pass_through", PassThrough(num_positions)
        )
        builder.ConnectInput("position", pos_pass.get_input_port())
        builder.ExportOutput(pos_pass.get_output_port(), "position_commanded")

        builder.ExportOutput(state_demux.get_output_port(0), "position_measured")
        builder.ExportOutput(state_demux.get_output_port(1), "velocity_estimated")
        state_pass = builder.AddNamedSystem(
            "state_pass_through", PassThrough(2 * num_positions)
        )
        builder.ConnectInput("state", state_pass.get_input_port())
        builder.ExportOutput(state_pass.get_output_port(), "state_estimated")
        builder.ExportOutput(inverse_dynamics.get_output_port(), "torque_commanded")
        # TODO torque_measured should come from the sim_plant not the controller
        builder.ExportOutput(inverse_dynamics.get_output_port(), "torque_measured")
        builder.ExportOutput(contact_forces.get_output_port(), "torque_external")

        builder.BuildInto(self)