from typing import Any, ClassVar, List, Optional
from types import SimpleNamespace
from pydrake.all import *
from .gen3_constants import *
import numpy as np
from .gen3_control import BuildGen3Control, AddSimGen3Driver
from robotiq_2f_85 import AddSim2f85Driver
from common import ConfigureParser

# Driver
class Gen3Driver:
    __fields__: ClassVar[tuple] = (
        SimpleNamespace(name="hand_model_name", type=str),
        SimpleNamespace(name="control_level", type=Gen3ControlLevel),
        SimpleNamespace(name="control_mode", type=Gen3JointControlMode),
        SimpleNamespace(name="ip_address", type=str),
        SimpleNamespace(name="port", type=int),
    )
    hand_model_name: str
    control_level: Gen3ControlLevel = Gen3ControlLevel.kHighLevel
    control_mode: Gen3JointControlMode = Gen3JointControlMode.kPosition
    ip_address: str = None
    port: int = None


# Driver Functions
def ApplyDriverConfig(
    driver_config: Gen3Driver,
    model_instance_name: str,
    sim_plant: MultibodyPlant,
    models_from_directives_map: List[ModelInstanceInfo],
    lcm_buses: LcmBuses,
    builder: DiagramBuilder,
):
    gen3_model: ModelInstanceInfo = models_from_directives_map[model_instance_name]
    robotiq_2f_85_model: ModelInstanceInfo  = models_from_directives_map[driver_config.hand_model_name]

    # Make arm controller plant
    controller_plant = MultibodyPlant(0.0)
    parser = Parser(controller_plant)
    controller_models: List[ModelInstanceIndex] = parser.AddModels(
        gen3_model.model_path
    )
    assert len(controller_models) == 1
    gen3_controller_model_idx = controller_models[0]

    controller_plant.WeldFrames(
        controller_plant.world_frame(),
        controller_plant.GetFrameByName(
            gen3_model.child_frame_name, gen3_controller_model_idx
        ),
        RigidTransform(),
    )

    # Make this more generic with tcp_frame or something
    X_ee = RigidTransform()
    X_ee.set_translation([0, 0, -0.0615250000000001])
    X_ee.set_rotation(RotationMatrix(RollPitchYaw([np.pi, 0, 0])))
    ee_frame = controller_plant.AddFrame(
        FixedOffsetFrame(
            "end_effector_frame",
            controller_plant.GetFrameByName("bracelet_no_vision_link"),
            X_ee,
            gen3_controller_model_idx,
        )
    )
    parser = Parser(controller_plant)
    ConfigureParser(parser)
    gripper = parser.AddModelsFromUrl(
        f"package://piplup_models/robotiq_description/sdf/robotiq_2f_85_static.sdf"
    )[0]
    controller_plant.WeldFrames(
        ee_frame,
        controller_plant.GetFrameByName("robotiq_arg2f_base_link", gripper),
        RigidTransform(RotationMatrix(RollPitchYaw([0, 0, np.pi / 2]))),
    )
    controller_plant.Finalize()

    gripper_controller_plant = MultibodyPlant(0.0)
    parser = Parser(gripper_controller_plant)
    controller_models: List[ModelInstanceIndex] = parser.AddModels(
        robotiq_2f_85_model.model_path
    )
    assert len(controller_models) == 1
    robotiq_2f_85_controller_model_idx = controller_models[0]

    gripper_controller_plant.WeldFrames(
        gripper_controller_plant.world_frame(),
        gripper_controller_plant.GetFrameByName(
            robotiq_2f_85_model.child_frame_name, robotiq_2f_85_controller_model_idx
        ),
        RigidTransform(),
    )
    gripper_controller_plant.Finalize()

    builder.AddNamedSystem(
        f"{model_instance_name}_controller_plant", SharedPointerSystem(controller_plant)
    )
    builder.AddNamedSystem(
        f"{driver_config.hand_model_name}_controller_plant", SharedPointerSystem(gripper_controller_plant)
    )

    if driver_config.ip_address and driver_config.port:
        pass  # TODO BuildGen3Control
    else:
        AddSimGen3Driver(
            sim_plant, gen3_model.model_instance, controller_plant, builder
        )

        if driver_config.hand_model_name == "2f_85":
            gripper_sys = AddSim2f85Driver(sim_plant, robotiq_2f_85_model.model_instance, gripper_controller_plant, builder)
            for i in range(gripper_sys.num_input_ports()):
                port = gripper_sys.get_input_port(i)
                if not builder.IsConnectedOrExported(port):
                    builder.ExportInput(port, f"{driver_config.hand_model_name}.{port.get_name()}")