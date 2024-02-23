#pragma once
#define _OS_UNIX

#include <BaseClientRpc.h>
#include <Common.pb.h>
#include <DeviceConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <drake/common/drake_copyable.h>
#include <drake/common/name_value.h>
#include <drake/common/scoped_singleton.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/leaf_system.h>

namespace piplup
{
    namespace kinova_gen3
    {
        using namespace drake;

        namespace k_api = Kinova::Api;
        class Gen3HardwareInterface : public systems::LeafSystem<double>
        {
        public:
            Gen3HardwareInterface();

        private:
            systems::EventStatus Initialize(const systems::Context<double> &,
                                            systems::State<double> *) const;
            void CalcUpdate(const systems::Context<double> &,
                            systems::State<double> *) const;
            k_api::Base::BaseClient * base_;
            k_api::SessionManager * session_manager;
            k_api::RouterClient * router;
            k_api::TransportClientTcp * transport;
        };

    } // namespace kinova_gen3
} // namespace piplup