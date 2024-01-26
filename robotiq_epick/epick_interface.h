#pragma once
#include <chrono>
#include <drake/common/drake_copyable.h>
#include <drake/common/name_value.h>
#include <drake/common/scoped_singleton.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/leaf_system.h>
#include <epick_driver_interface/default_driver.hpp>
#include <epick_driver_interface/default_driver_utils.hpp>
#include <epick_driver_interface/default_serial.hpp>
#include <iostream>
#include <memory>
#include <vector>
namespace piplup
{
    namespace epick
    {
        using namespace drake;

        struct EPickInterfaceConfig
        {
            std::string serial_port{"/dev/ttyUSB1"};
            int baud_rate = 115200;
            double timeout = 0.5;
            int slave_address = 0x09;
            template<typename Archive>
            void Serialize(Archive * a)
            {
                a->Visit(DRAKE_NVP(serial_port));
                a->Visit(DRAKE_NVP(baud_rate));
                a->Visit(DRAKE_NVP(timeout));
                a->Visit(DRAKE_NVP(slave_address));
            }
        };

        class EPickInterface : public systems::LeafSystem<double>
        {
        public:
            EPickInterface(const EPickInterfaceConfig & epick_config);

            void SendCommandAndReceiveStatus(const systems::Context<double> & context,
                                             systems::State<double> * state) const;

        private:
            std::unique_ptr<epick_driver::DefaultDriver> driver_;
            std::unique_ptr<epick_driver::DefaultSerial> serial_;
        };
    } // namespace epick
} // namespace piplup