#include "kinova_gen3/gen3_interface.h"

namespace piplup
{
    namespace kinova_gen3
    {
        using namespace drake;
        namespace k_api = Kinova::Api;
        Gen3HardwareInterface::Gen3HardwareInterface()
        {
            double hz = 40.0;
            DeclareInitializationUnrestrictedUpdateEvent(
                &Gen3HardwareInterface::Initialize);
            DeclarePeriodicUnrestrictedUpdateEvent(
                1 / hz, 0.0, &Gen3HardwareInterface::CalcUpdate);
        }
        systems::EventStatus Gen3HardwareInterface::Initialize(
            const systems::Context<double> &, systems::State<double> *)
        {
            auto error_callback = [](k_api::KError err) {
                cout << "_________ callback error _________" << err.toString();
            };
            transport = new k_api::TransportClientTcp();
            router = new k_api::RouterClient(transport, error_callback);
            transport->connect("192.168.1.10", 10000);

            // Set session data connection information
            auto create_session_info = k_api::Session::CreateSessionInfo();
            create_session_info.set_username("admin");
            create_session_info.set_password("admin");
            create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
            create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

            // Session manager service wrapper
            std::cout << "Creating session for communication" << std::endl;
            session_manager = new k_api::SessionManager(router);
            session_manager->CreateSession(create_session_info);
            std::cout << "Session created" << std::endl;

            // Create services
            base_ = new k_api::Base::BaseClient(router);
            return systems::EventStatus::Succeeded();
        }

        void Gen3HardwareInterface::CalcUpdate(const systems::Context<double> &,
                                               systems::State<double> *) const
        {}
    } // namespace kinova_gen3
} // namespace piplup