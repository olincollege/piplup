#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "kinova_gen3/gen3_interface.h"

namespace piplup
{
    namespace kinova_gen3
    {

        using namespace drake;

        int DoMain()
        {
            drake::logging::set_log_level("debug");
            systems::DiagramBuilder<double> builder;
            auto gen3_interface = builder.AddNamedSystem<Gen3HardwareInterface>(
                "gen3", "", "", Gen3HandType::k2f85);

            auto diagram = builder.Build();
            systems::Simulator<double> simulator(*diagram);
            auto & sim_context = simulator.get_mutable_context();
            auto & interface_context =
                gen3_interface->GetMyMutableContextFromRoot(&sim_context);
            simulator.set_target_realtime_rate(1.0);

            gen3_interface->GetInputPort("command").FixValue(
                &interface_context,
                // (VectorX<double>(7) << 0.2, 0.477, 0.0, 1.32, 0.0, 1.33,
                // 0.0).finished()
                VectorX<double>::Zero(7));
            gen3_interface->GetInputPort("control_mode")
                .FixValue(&interface_context, Gen3ControlMode::kTwist);

            gen3_interface->GetInputPort("2f_85.command")
                .FixValue(&interface_context, 0.2);
            simulator.Initialize();
            simulator.AdvanceTo(simulator.get_context().get_time() + 2);
            gen3_interface->GetInputPort("2f_85.command")
                .FixValue(&interface_context, 0.5);
            // auto position = gen3_interface->GetOutputPort("position_measured")
            //                     .Eval(interface_context);
            // drake::log()->info("Position: {}", position);
            simulator.AdvanceTo(simulator.get_context().get_time() + 2);
            drake::log()->info("Realtime Rate: {}", simulator.get_actual_realtime_rate());
            return 0;
        }
    } // namespace kinova_gen3
} // namespace piplup

int main()
{
    return piplup::kinova_gen3::DoMain();
}