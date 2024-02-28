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
            
            // DeclareAbstractState(Value<std::future<k_api::BaseCyclic::Feedback>>());
            // Input Ports
            command_port_ = &DeclareVectorInputPort("command", 7);
            control_mode_port_ =
                &DeclareAbstractInputPort("control_mode", Value<Gen3ControlMode>());

            if(hand_type_ == Gen3HandType::k2f85)
            {
                hand_command_port_ = &DeclareVectorInputPort("2f_85.command", 1);
            }

            // Output Ports
            DeclareStateOutputPort("pose_measured", pose_measured_state_index_);
            DeclareStateOutputPort("twist_measured", twist_measured_state_index_);
            DeclareStateOutputPort("wrench_measured", wrench_measured_state_index_);
            DeclareStateOutputPort("position_measured", position_measured_state_index_);
            DeclareStateOutputPort("velocity_measured", velocity_measured_state_index_);
            DeclareStateOutputPort("torque_measured", torque_measured_state_index_);

            // Update Events
            double hz = 40.0;
            DeclareInitializationUnrestrictedUpdateEvent(
                &Gen3HardwareInterface::Initialize);
            DeclarePeriodicUnrestrictedUpdateEvent(
                1 / hz, 0.0, &Gen3HardwareInterface::CalcUpdate);
            auto error_callback = [](k_api::KError err) {
                cout << "_________ callback error _________" << err.toString();
            };
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
            std::cout << "Creating session for communication" << std::endl;
            session_manager_ = new k_api::SessionManager(router_);
            session_manager_->CreateSession(create_session_info);
            std::cout << "Session created" << std::endl;

            // Create services
            base_ = new k_api::Base::BaseClient(router_);
            base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router_);
        }
        systems::EventStatus Gen3HardwareInterface::Initialize(
            const systems::Context<double> &, systems::State<double> *) const
        {
            if(base_->GetArmState().active_state()
               != k_api::Common::ArmState::ARMSTATE_SERVOING_READY)
            {
                return systems::EventStatus::Failed(this, "Arm not in ready state.");
            }
            return systems::EventStatus::Succeeded();
        }

        void Gen3HardwareInterface::CalcUpdate(const systems::Context<double> & context,
                                               systems::State<double> * state) const
        {
            
            const auto & command = command_port_->Eval<BasicVector<double>>(context);
            const auto & control_mode =
                control_mode_port_->Eval<Gen3ControlMode>(context);
            if(hand_type_ == Gen3HandType::k2f85)
            {
                const auto & hand_command =
                    hand_command_port_->Eval<BasicVector<double>>(context);
                k_api::Base::GripperCommand gripper_command;
                gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);

                auto finger = gripper_command.mutable_gripper()->add_finger();
                finger->set_finger_identifier(1);
                finger->set_value(hand_command[0]);
                base_->SendGripperCommand(gripper_command);
            }

            switch(control_mode)
            {
            case Gen3ControlMode::kTwist:
            {
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
                base_->SendTwistCommand_async(twist_command);
                break;
            }
            default:
                drake::log()->error("Gen3 control mode not implemented");
            }
        }

        Gen3HardwareInterface::~Gen3HardwareInterface()
        {
            session_manager_->CloseSession();

            // Deactivate the router and cleanly disconnect from the transport object
            router_->SetActivationStatus(false);
            transport_->disconnect();

            // Destroy the API
            delete base_;
            delete session_manager_;
            delete router_;
            delete transport_;
        }
    } // namespace kinova_gen3
} // namespace piplup