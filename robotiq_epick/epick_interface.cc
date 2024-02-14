#include "robotiq_epick/epick_interface.h"

namespace piplup
{
    namespace epick
    {
        using namespace drake;
        EPickInterface::EPickInterface(const EPickInterfaceConfig & epick_config)
        {
            auto serial = std::make_unique<epick_driver::DefaultSerial>();
            serial->set_port(epick_config.serial_port);
            serial->set_baudrate(epick_config.baud_rate);
            serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(epick_config.timeout)));

            driver_ = std::make_unique<epick_driver::DefaultDriver>(std::move(serial));
            driver_->set_slave_address(epick_config.slave_address);
            driver_->set_mode(epick_driver::GripperMode::AdvancedMode);
            driver_->set_grip_max_vacuum_pressure(-100);
            if(!driver_->connect())
            {
                throw std::runtime_error(
                    fmt::format("Failed to connect to EPick on serial port: {}",
                                epick_config.serial_port));
            }
            driver_->activate();

            pressure_state_idx_ = this->DeclareDiscreteState(1);
            object_det_state_idx_ = this->DeclareAbstractState(Value<bool>());
            this->DeclareAbstractInputPort("command", Value<bool>());
            this->DeclarePeriodicUnrestrictedUpdateEvent(
                0.0001, 0.0, &EPickInterface::SendCommandAndReceiveStatus);
            this->DeclareStateOutputPort("actual_vacuum_pressure", pressure_state_idx_);
            this->DeclareStateOutputPort("object_detection_status",
                                         object_det_state_idx_);
        }

        void EPickInterface::SendCommandAndReceiveStatus(
            const systems::Context<double> & context,
            systems::State<double> * state) const
        {
            auto command = get_input_port(0).Eval<bool>(context);
            if(command)
            {
                driver_->grip();
            }
            else
            {
                driver_->release();
            }
            auto status = driver_->get_status();
            auto & pressure_state =
                state->get_mutable_discrete_state(pressure_state_idx_);
            pressure_state[0] = (double)status.actual_vacuum_pressure;
            auto & obj_det_state =
                state->get_mutable_abstract_state<bool>(object_det_state_idx_);
            obj_det_state =
                status.object_detection_status
                == epick_driver::ObjectDetectionStatus::ObjectDetectedAtMinPressure;
        }

        void EPickInterface::release()
        {
            driver_->release();
        }
    } // namespace epick
} // namespace piplup