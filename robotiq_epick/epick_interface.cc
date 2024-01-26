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
            if(!driver_->connect())
            {
                throw std::runtime_error(
                    fmt::format("Failed to connect to EPick on serial port: {}",
                                epick_config.serial_port));
            }
            this->DeclareAbstractInputPort("command", Value<bool>());
            this->DeclarePeriodicUnrestrictedUpdateEvent(
                0.01, 0.0, &EPickInterface::SendCommandAndReceiveStatus);
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
        }
    } // namespace epick
} // namespace piplup