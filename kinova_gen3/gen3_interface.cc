#include "kinova_gen3/gen3_interface.h"

namespace piplup
{
    namespace kinova_gen3
    {
        using namespace drake;
        using systems::BasicVector;
        namespace k_api = Kinova::Api;
        Gen3HardwareInterface::Gen3HardwareInterface(std::string ip_address,
                                                     std::string port,
                                                     Gen3HandType hand_type)
          : hand_type_(hand_type)
        {
            // State
            pose_measured_state_index_ = DeclareDiscreteState(6);
            twist_measured_state_index_ = DeclareDiscreteState(6);
            wrench_measured_state_index_ = DeclareDiscreteState(6);
            position_measured_state_index_ = DeclareDiscreteState(7);
            velocity_measured_state_index_ = DeclareDiscreteState(7);
            torque_measured_state_index_ = DeclareDiscreteState(7);
            feedback_future_index_ = DeclareAbstractState(
                Value<std::shared_future<k_api::BaseCyclic::Feedback>>());
            gripper_future_index_ =
                DeclareAbstractState(Value<std::shared_future<void>>());
            arm_future_index_ = DeclareAbstractState(Value<std::shared_future<void>>());

            // DeclareAbstractState(Value<std::future<k_api::BaseCyclic::Feedback>>());
            // Input Ports
            command_port_ = &DeclareVectorInputPort("command", 7);
            control_mode_port_ =
                &DeclareAbstractInputPort("control_mode", Value<Gen3ControlMode>());

            if(hand_type_ == Gen3HandType::k2f85)
            {
                hand_command_port_ =
                    &DeclareVectorInputPort("2f_85.command", BasicVector<double>({0.0}));
            }

            // Output Ports
            DeclareStateOutputPort("pose_measured", pose_measured_state_index_);
            DeclareStateOutputPort("twist_measured", twist_measured_state_index_);
            DeclareStateOutputPort("wrench_measured", wrench_measured_state_index_);
            DeclareStateOutputPort("position_measured", position_measured_state_index_);
            DeclareStateOutputPort("velocity_measured", velocity_measured_state_index_);
            DeclareStateOutputPort("torque_measured", torque_measured_state_index_);

            // Update Events
            double hz = 20.0;
            DeclareInitializationUnrestrictedUpdateEvent(
                &Gen3HardwareInterface::Initialize);
            // DeclarePeriodicPublishEvent(1 / hz, 0.0,
            // &Gen3HardwareInterface::CalcUpdate);
            DeclarePeriodicUnrestrictedUpdateEvent(
                1 / hz, 0.0, &Gen3HardwareInterface::CalcUpdate);
            auto error_callback = [](k_api::KError err) {
                cout << "_________ callback error _________" << err.toString();
            };
            drake::log()->info("Connecting to transport client...");
            transport_ = new k_api::TransportClientTcp();
            router_ = new k_api::RouterClient(transport_, error_callback);
            transport_->connect("192.168.1.10", 10000);

            // Set session data connection information
            auto create_session_info = k_api::Session::CreateSessionInfo();
            create_session_info.set_username("admin");
            create_session_info.set_password("admin");
            create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
            create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

            // Session manager service wrapper
            drake::log()->info("Creating session for communication");
            session_manager_ = new k_api::SessionManager(router_);
            session_manager_->CreateSession(create_session_info);
            drake::log()->info("Session created");
            // Create services
            base_ = new k_api::Base::BaseClient(router_);
            base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router_);
        }
        systems::EventStatus Gen3HardwareInterface::Initialize(
            const systems::Context<double> & context,
            systems::State<double> * state) const
        {
            auto servoing_mode = k_api::Base::ServoingModeInformation();
            servoing_mode.set_servoing_mode(
                k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            base_->SetServoingMode(servoing_mode);
            if(base_->GetArmState().active_state()
               != k_api::Common::ArmState::ARMSTATE_SERVOING_READY)
            {
                return systems::EventStatus::Failed(this, "Arm not in ready state.");
            }
            return systems::EventStatus::Succeeded();
        }

        void Gen3HardwareInterface::DoCalcNextUpdateTime(
            const systems::Context<double> & context,
            systems::CompositeEventCollection<double> * events, double * time) const
        {
            systems::LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
            auto callback = [](const System<double> & system,
                               const systems::Context<double> & callback_context,
                               const systems::UnrestrictedUpdateEvent<double> &,
                               systems::State<double> * callback_state) {
                const auto & self = dynamic_cast<const Gen3HardwareInterface &>(system);
                auto & feedback_future = callback_state->get_abstract_state<
                    std::shared_future<k_api::BaseCyclic::Feedback>>(
                    self.feedback_future_index_);
                if(!feedback_future.valid()
                   || feedback_future.wait_for(std::chrono::seconds(0))
                          == std::future_status::ready)
                {
                    drake::log()->debug("Feedback Time: {}", callback_context.get_time());
                    if (feedback_future.valid()) {
                        self.UpdateState(feedback_future.get(), callback_state);
                    }
                    auto future = self.base_cyclic_->RefreshFeedback_async().share();
                    callback_state->get_mutable_abstract_state<
                        std::shared_future<k_api::BaseCyclic::Feedback>>(
                        self.feedback_future_index_) = future;
                }
                else
                {
                    return systems::EventStatus::DidNothing();
                }
                return systems::EventStatus::Succeeded();
            };
            *time = context.get_time() + 1.0 / 20.0;
            systems::EventCollection<systems::UnrestrictedUpdateEvent<double>> &
                uu_events = events->get_mutable_unrestricted_update_events();
            uu_events.AddEvent(systems::UnrestrictedUpdateEvent<double>(
                systems::TriggerType::kTimed, callback));
        }

        void Gen3HardwareInterface::CalcUpdate(const systems::Context<double> & context,
                                               systems::State<double> * state) const
        {
            auto gripper_future = state->get_abstract_state<std::shared_future<void>>(
                gripper_future_index_);
            auto arm_future =
                state->get_abstract_state<std::shared_future<void>>(arm_future_index_);
            if(hand_type_ == Gen3HandType::k2f85
               && (!gripper_future.valid()
                   || gripper_future.wait_for(std::chrono::seconds(0))
                          == std::future_status::ready))
            {
                const auto & hand_command =
                    hand_command_port_->Eval<BasicVector<double>>(context);

                k_api::Base::GripperCommand gripper_command;
                auto gripper_command_mode = k_api::Base::GRIPPER_POSITION;
                gripper_command.set_mode(gripper_command_mode);
                drake::log()->debug("Sending Gripper Command {} \t Mode: {}",
                                    hand_command,
                                    gripper_command_mode);
                auto finger = gripper_command.mutable_gripper()->add_finger();
                finger->set_finger_identifier(1);
                finger->set_value(hand_command[0]);

                auto future = base_->SendGripperCommand_async(gripper_command).share();
                state->get_mutable_abstract_state<std::shared_future<void>>(
                    gripper_future_index_) = future;
            }

            const auto & command = command_port_->Eval<BasicVector<double>>(context);
            const auto & control_mode =
                control_mode_port_->Eval<Gen3ControlMode>(context);

            if(!arm_future.valid()
               || arm_future.wait_for(std::chrono::seconds(0))
                      == std::future_status::ready)
            {

                switch(control_mode)
                {
                case Gen3ControlMode::kTwist:
                {
                    drake::log()->debug("Sending Twist Command {}", command);
                    auto twist_command = k_api::Base::TwistCommand();
                    twist_command.set_reference_frame(
                        k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
                    twist_command.set_duration(0);
                    auto twist = twist_command.mutable_twist();
                    twist->set_angular_x(command[0]);
                    twist->set_angular_y(command[1]);
                    twist->set_angular_z(command[2]);
                    twist->set_linear_x(command[3]);
                    twist->set_linear_y(command[4]);
                    twist->set_linear_z(command[5]);
                    auto future = base_->SendTwistCommand_async(twist_command).share();
                    state->get_mutable_abstract_state<std::shared_future<void>>(
                        arm_future_index_) = future;
                    break;
                }
                default:
                    drake::log()->error("Gen3 control mode not implemented");
                }
            }
        }

        void Gen3HardwareInterface::UpdateState(
            const k_api::BaseCyclic::Feedback & feedback,
            systems::State<double> * state) const
        {
            for(int i = 0; i < feedback.actuators_size(); i++)
            {
                // drake::log()->debug(
                //     "Actuator {} \t Position: {} \t Velocity: {} \t Torque: {}",
                //     i,
                //     feedback.actuators(i).position(),
                //     feedback.actuators(i).velocity(),
                //     feedback.actuators(i).torque());
                state->get_mutable_discrete_state(position_measured_state_index_)
                    .SetAtIndex(i, feedback.actuators(i).position() * (M_PI / 180.0));
                state->get_mutable_discrete_state(velocity_measured_state_index_)
                    .SetAtIndex(i, feedback.actuators(i).velocity() * (M_PI / 180.0));
                state->get_mutable_discrete_state(torque_measured_state_index_)
                    .SetAtIndex(i, feedback.actuators(i).torque() * (M_PI / 180.0));
            }
            state->get_mutable_discrete_state(pose_measured_state_index_)
                .SetAtIndex(0, feedback.base().tool_pose_theta_x() * (M_PI / 180.0));
            state->get_mutable_discrete_state(pose_measured_state_index_)
                .SetAtIndex(1, feedback.base().tool_pose_theta_y() * (M_PI / 180.0));
            state->get_mutable_discrete_state(pose_measured_state_index_)
                .SetAtIndex(2, feedback.base().tool_pose_theta_z() * (M_PI / 180.0));
            state->get_mutable_discrete_state(pose_measured_state_index_)
                .SetAtIndex(3, feedback.base().tool_pose_x());
            state->get_mutable_discrete_state(pose_measured_state_index_)
                .SetAtIndex(4, feedback.base().tool_pose_y());
            state->get_mutable_discrete_state(pose_measured_state_index_)
                .SetAtIndex(5, feedback.base().tool_pose_z());

            state->get_mutable_discrete_state(twist_measured_state_index_)
                .SetAtIndex(0, feedback.base().tool_twist_angular_x() * (M_PI / 180.0));
            state->get_mutable_discrete_state(twist_measured_state_index_)
                .SetAtIndex(1, feedback.base().tool_twist_angular_y() * (M_PI / 180.0));
            state->get_mutable_discrete_state(twist_measured_state_index_)
                .SetAtIndex(2, feedback.base().tool_twist_angular_z() * (M_PI / 180.0));
            state->get_mutable_discrete_state(twist_measured_state_index_)
                .SetAtIndex(3, feedback.base().tool_twist_linear_x());
            state->get_mutable_discrete_state(twist_measured_state_index_)
                .SetAtIndex(4, feedback.base().tool_twist_linear_y());
            state->get_mutable_discrete_state(twist_measured_state_index_)
                .SetAtIndex(5, feedback.base().tool_twist_linear_z());

            state->get_mutable_discrete_state(wrench_measured_state_index_)
                .SetAtIndex(
                    0, feedback.base().tool_external_wrench_torque_x() * (M_PI / 180.0));
            state->get_mutable_discrete_state(wrench_measured_state_index_)
                .SetAtIndex(
                    1, feedback.base().tool_external_wrench_torque_y() * (M_PI / 180.0));
            state->get_mutable_discrete_state(wrench_measured_state_index_)
                .SetAtIndex(
                    2, feedback.base().tool_external_wrench_torque_z() * (M_PI / 180.0));
            state->get_mutable_discrete_state(wrench_measured_state_index_)
                .SetAtIndex(3, feedback.base().tool_external_wrench_force_x());
            state->get_mutable_discrete_state(wrench_measured_state_index_)
                .SetAtIndex(4, feedback.base().tool_external_wrench_force_y());
            state->get_mutable_discrete_state(wrench_measured_state_index_)
                .SetAtIndex(5, feedback.base().tool_external_wrench_force_z());
        }

        Gen3HardwareInterface::~Gen3HardwareInterface()
        {
            drake::log()->info("Closing Kinova Interface...");
            session_manager_->CloseSession();

            // Deactivate the router and cleanly disconnect from the transport object
            router_->SetActivationStatus(false);
            transport_->disconnect();

            // Destroy the API
            delete base_;
            delete session_manager_;
            delete router_;
            delete transport_;
            drake::log()->info("Kinova Interface Closed");
        }
    } // namespace kinova_gen3
} // namespace piplup