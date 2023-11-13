
#include "realsense/realsensed400.h"
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
    namespace realsense
    {

        PYBIND11_MODULE(realsensed400, m)
        {
            m.doc() = "Realsense D400 python bindings.";

            py::module::import("pydrake.systems.framework");

            py::class_<RealSenseD400, LeafSystem<double>>(m, "RealSenseD400")
                .def(py::init<std::string, const systems::sensors::CameraConfig &>(),
                     py::arg("device_serial_number"),
                     py::arg("camera_config"));
            {
                using Class = RealsenseInterfaceConfig;
                py::class_<Class> cls(
                    m, "RealsenseInterfaceConfig", "Realsense Interface Config");
                cls.def(ParamInit<Class>());
                DefAttributesUsingSerialize(&cls);
                DefReprUsingSerialize(&cls);
                DefCopyAndDeepCopy(&cls);
            }
        }
    } // namespace realsense
} // namespace piplup