#include <drake/bindings/pydrake/common/cpp_template_pybind.h>
#include <drake/bindings/pydrake/common/default_scalars_pybind.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_state.h>
#include <pybind11/pybind11.h>
#include <functional>

#include "robotiq_epick/suction_force_model.h"
namespace py = pybind11;

using drake::pydrake::DefineTemplateClassWithDefault;
using drake::pydrake::GetPyParam;
using drake::systems::LeafSystem;
namespace piplup {
namespace suction_gripper {

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
        .def(py::init<double, double, const drake::multibody::BodyIndex&,
                      const std::pair<drake::geometry::FrameId, std::vector<drake::math::RigidTransformd>> &,
                      const std::vector<drake::geometry::GeometryIdSet>&,
                      const std::unordered_map<drake::geometry::GeometryId,
                                               drake::multibody::BodyIndex>&>(),
             py::arg("time_step"), py::arg("suction_cup_area"),
             py::arg("gripper_body"),
             py::arg("action_point_frames"),
             py::arg("edge_points"),
             py::arg("obj_geom_id_to_body_idx_map"))
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
}
}  // namespace suction_gripper
}  // namespace piplup