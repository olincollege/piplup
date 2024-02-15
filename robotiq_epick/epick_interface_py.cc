
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "robotiq_epick/epick_driver_interface/driver.hpp"
#include "robotiq_epick/epick_interface.h"
#include <drake/bindings/pydrake/common/cpp_template_pybind.h>
#include <drake/bindings/pydrake/common/default_scalars_pybind.h>
#include <drake/bindings/pydrake/common/serialize_pybind.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_state.h>
#include <functional>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using drake::pydrake::DefAttributesUsingSerialize;
using drake::pydrake::DefCopyAndDeepCopy;
using drake::pydrake::DefineTemplateClassWithDefault;
using drake::pydrake::DefReprUsingSerialize;
using drake::pydrake::GetPyParam;
using drake::pydrake::ParamInit;
using drake::systems::LeafSystem;
namespace piplup
{
    namespace epick
    {

        PYBIND11_MODULE(epick_interface, m)
        {
            m.doc() = "Robotiq EPick python bindings.";

            py::module::import("pydrake.systems.framework");

            py::class_<EPickInterface, LeafSystem<double>>(m, "EPickInterface")
                .def(py::init<const EPickInterfaceConfig &>(), py::arg("epick_config"))
                .def("release", &EPickInterface::release);
            {
                using Class = EPickInterfaceConfig;
                py::class_<Class> cls(
                    m, "EPickInterfaceConfig", "EPick Interface Config");
                cls.def(ParamInit<Class>());
                DefAttributesUsingSerialize(&cls);
                DefReprUsingSerialize(&cls);
                DefCopyAndDeepCopy(&cls);
            }

            py::enum_<epick_driver::ObjectDetectionStatus>(m, "ObjectDetectionStatus")
                .value("ObjectDetectedAtMinPressure",
                       epick_driver::ObjectDetectionStatus::ObjectDetectedAtMinPressure)
                .value("ObjectDetectedAtMaxPressure",
                       epick_driver::ObjectDetectionStatus::ObjectDetectedAtMaxPressure)
                .value("NoObjectDetected",
                       epick_driver::ObjectDetectionStatus::NoObjectDetected)
                .value("Unknown", epick_driver::ObjectDetectionStatus::Unknown)
                .export_values();
        }
    } // namespace epick
} // namespace piplup