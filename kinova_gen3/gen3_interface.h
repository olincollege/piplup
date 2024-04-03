#pragma once
#define _OS_UNIX

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <Common.pb.h>
#include <DeviceConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <cmath>
#include <drake/common/drake_copyable.h>
#include <drake/common/name_value.h>
#include <drake/common/scoped_singleton.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/event.h>
#include <drake/systems/framework/leaf_system.h>

namespace piplup
{
    namespace kinova_gen3
    {
        using namespace drake;
        namespace k_api = Kinova::Api;

        enum class Gen3ControlMode
        {
            kPosition,
            kVelocity,
            kPose,
            kTwist
        };

        enum class Gen3HandType
        {
            kNone,
            k2f85,
            kEPick
        };

        struct Gen3InterfaceConfig
        {
            std::string ip_address;
            std::string port;
            template<typename Archive>
            void Serialize(Archive * a)
            {
                a->Visit(DRAKE_NVP(ip_address));
                a->Visit(DRAKE_NVP(port));
            }
        };

        class Gen3HardwareInterface : public systems::LeafSystem<double>
        {
        public:
            Gen3HardwareInterface(std::string, std::string, Gen3HandType);
            ~Gen3HardwareInterface();

            // private:
            systems::EventStatus Initialize(const systems::Context<double> &,
                                            systems::State<double> *) const;
            void CalcUpdate(const systems::Context<double> & context,
                            systems::State<double> * state) const;
            void DoCalcNextUpdateTime(const systems::Context<double> & context,
                                      systems::CompositeEventCollection<double> * events,
                                      double * time) const final;
            void UpdateState(const k_api::BaseCyclic::Feedback & feedback,
                             systems::State<double> * state) const;

            k_api::Base::BaseClient * base_;
            k_api::BaseCyclic::BaseCyclicClient * base_cyclic_;
            k_api::SessionManager * session_manager_;
            k_api::RouterClient * router_;
            k_api::TransportClientTcp * transport_;
            systems::InputPort<double> * command_port_;
            systems::InputPort<double> * control_mode_port_;
            systems::InputPort<double> * hand_command_port_;
            Gen3HandType hand_type_;
            systems::DiscreteStateIndex pose_measured_state_index_;
            systems::DiscreteStateIndex twist_measured_state_index_;
            systems::DiscreteStateIndex wrench_measured_state_index_;
            systems::DiscreteStateIndex position_measured_state_index_;
            systems::DiscreteStateIndex velocity_measured_state_index_;
            systems::DiscreteStateIndex torque_measured_state_index_;
            systems::AbstractStateIndex feedback_future_index_;
            systems::AbstractStateIndex gripper_future_index_;
            systems::AbstractStateIndex twist_future_index_;
            systems::AbstractStateIndex position_future_index_;
            systems::AbstractStateIndex promise_notification_handle_index_;
        };

    } // namespace kinova_gen3
} // namespace piplup