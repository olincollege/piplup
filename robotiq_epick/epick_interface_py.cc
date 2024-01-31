
#include "drake/bindings/pydrake/common/value_pybind.h"
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
                .def(py::init<const EPickInterfaceConfig &>(), py::arg("epick_config"));
            {
                using Class = EPickInterfaceConfig;
                py::class_<Class> cls(
                    m, "EPickInterfaceConfig", "EPick Interface Config");
                cls.def(ParamInit<Class>());
                DefAttributesUsingSerialize(&cls);
                DefReprUsingSerialize(&cls);
                DefCopyAndDeepCopy(&cls);
            }

            {
                using Class = epick_driver::GripperStatus;
                py::class_<Class> cls(m, "GripperStatus", "EPick Gripper Status");
                cls.def(ParamInit<Class>());
                cls.def_readwrite("max_vacuum_pressure",
                                  &epick_driver::GripperStatus::max_vacuum_pressure);
                cls.def_readwrite("actual_vacuum_pressure",
                                  &epick_driver::GripperStatus::actual_vacuum_pressure);
                DefCopyAndDeepCopy(&cls);
            }
            drake::pydrake::AddValueInstantiation<epick_driver::GripperStatus>(m);
        }
    } // namespace epick
} // namespace piplup