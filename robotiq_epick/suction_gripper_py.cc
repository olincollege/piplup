#include <drake/bindings/pydrake/common/cpp_template_pybind.h>
#include <drake/bindings/pydrake/common/default_scalars_pybind.h>
#include <pybind11/pybind11.h>

#include "robotiq_epick/example_gripper_multibody_model.h"
#include "robotiq_epick/suction_force_model.h"

namespace py = pybind11;

using drake::pydrake::DefineTemplateClassWithDefault;
using drake::pydrake::GetPyParam;
using drake::systems::LeafSystem;

namespace drake::examples::multibody::suction_gripper {
namespace {

PYBIND11_MODULE(suction_gripper, m) {
    m.doc() = "Suction vacuum gripper python bindings.";

    py::module::import("pydrake.systems.framework");

    py::class_<CupPressureSource, LeafSystem<double>>(m, "CupPressureSource")
        .def(py::init<double, double, int>(), py::arg("vacuum_source_pressure"),
             py::arg("max_suction_dist"), py::arg("num_suction_cups"))
        .def("GetCupObjDistInputPort",
             &CupPressureSource::GetCupObjDistInputPort,
             py::return_value_policy::reference)
        .def("GetSuctionCmdInputPort",
             &CupPressureSource::GetSuctionCmdInputPort,
             py::return_value_policy::reference)
        .def("GetSuctionCupPressureOutputPort",
             &CupPressureSource::GetSuctionCupPressureOutputPort,
             py::return_value_policy::reference);

    py::class_<CupObjInterface, LeafSystem<double>>(m, "CupObjInterface")
        .def(py::init<
             double, double, const std::vector<drake::geometry::GeometryId>&,
             const std::unordered_map<drake::geometry::GeometryId,
                                      drake::multibody::BodyIndex>&,
             const std::vector<std::vector<drake::geometry::GeometryId>>&,
             const std::unordered_map<drake::geometry::GeometryId,
                                      drake::multibody::BodyIndex>&>(),
                                      py::arg("time_step"),
                                      py::arg("suction_cup_area"),
                                      py::arg("suction_cup_act_pt_geom_id_vec"),
                                      py::arg("suction_cup_act_pt_geom_id_to_body_idx_map"),
                                      py::arg("suction_cup_edge_pt_geom_id_vec"),
                                      py::arg("obj_geom_id_to_body_idx_map")
                                      )
        .def("GetGeomQueryInputPort", &CupObjInterface::GetGeomQueryInputPort,
             py::return_value_policy::reference)
        .def("GetSuctionCupPressureInputPort",
             &CupObjInterface::GetSuctionCupPressureInputPort,
             py::return_value_policy::reference)
        .def("GetSuctionForceOutputPort",
             &CupObjInterface::GetSuctionForceOutputPort,
             py::return_value_policy::reference)
        .def("GetCupObjDistOutputPort",
             &CupObjInterface::GetCupObjDistOutputPort,
             py::return_value_policy::reference);

    py::class_<ExampleGripperMultibodyModel>(m, "ExampleGripperMultibodyModel")
        .def(py::init<drake::multibody::MultibodyPlant<double>*,
                      const drake::multibody::Body<double>&>(),
             py::arg("plant"), py::arg("wrist_body"))
        .def("get_gripper_model_instance",
             &ExampleGripperMultibodyModel::get_gripper_model_instance)
        .def("get_suction_cup_act_pt_geom_id_vec",
             &ExampleGripperMultibodyModel::get_suction_cup_act_pt_geom_id_vec)
        .def("get_suction_cup_act_pt_geom_id_to_body_idx_map",
             &ExampleGripperMultibodyModel::
                 get_suction_cup_act_pt_geom_id_to_body_idx_map)
        .def("get_suction_cup_edge_pt_geom_id_vec",
             &ExampleGripperMultibodyModel::get_suction_cup_edge_pt_geom_id_vec)
        .def("CalcCupArea", &ExampleGripperMultibodyModel::CalcCupArea);
}
}  // namespace
}  // namespace drake::examples::multibody::suction_gripper