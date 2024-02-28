#include "kinova_gen3/gen3_interface.h"
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
    namespace kinova_gen3
    {

        PYBIND11_MODULE(gen3_interface, m)
        {
            m.doc() = "Kinova Gen3 python bindings.";

            py::module::import("pydrake.systems.framework");
            {
                using Class = Gen3HandType;
                py::enum_<Class>(m, "Gen3HandType")
                    .value("kNone", Class::kNone)
                    .value("k2f85", Class::k2f85)
                    .value("kEPick", Class::kEPick);
            }

            {
                using Class = Gen3ControlMode;
                py::enum_<Class>(m, "Gen3ControlMode")
                    .value("kPose", Class::kPose)
                    .value("kTwist", Class::kTwist)
                    .value("kPosition", Class::kPosition)
                    .value("kVelocity", Class::kVelocity);
            }

            py::class_<Gen3HardwareInterface, LeafSystem<double>>(m,
                                                                  "Gen3HardwareInterface")
                .def(py::init<std::string, std::string, Gen3HandType>(),
                     py::arg("ip_address"),
                     py::arg("port"),
                     py::arg("hand_type"));
            {
                using Class = Gen3InterfaceConfig;
                py::class_<Class> cls(m, "Gen3InterfaceConfig", "Gen3 Interface Config");
                cls.def(ParamInit<Class>());
                DefAttributesUsingSerialize(&cls);
                DefReprUsingSerialize(&cls);
                DefCopyAndDeepCopy(&cls);
            }
        }
    } // namespace kinova_gen3
} // namespace piplup