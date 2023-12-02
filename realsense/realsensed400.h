#pragma once
#include <drake/common/drake_copyable.h>
#include <drake/common/name_value.h>
#include <drake/common/scoped_singleton.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/sensors/camera_config.h>
#include <drake/systems/sensors/image.h>
#include <iostream>
#include <librealsense2/rs.hpp>

namespace piplup
{
    namespace realsense
    {
        using namespace drake;

        struct RealsenseInterfaceConfig
        {
            std::string serial_number;
            std::string json_config_file{""};
            template<typename Archive>
            void Serialize(Archive * a)
            {
                a->Visit(DRAKE_NVP(serial_number));
                a->Visit(DRAKE_NVP(json_config_file));
            }
        };

        class RealSenseD400 : public systems::LeafSystem<double>
        {
        public:
            RealSenseD400(std::string device_serial_number,
                          const systems::sensors::CameraConfig & camera_config);

            void PollForImages(const systems::Context<double> & context,
                               systems::State<double> * state) const;

        private:
            systems::AbstractStateIndex color_state_idx_;
            systems::AbstractStateIndex depth_state_idx_;
            std::shared_ptr<rs2::context> context_;
            rs2::pipeline pipeline_;
            const int depth_width_;
            const int depth_height_;
            const int color_width_;
            const int color_height_;
        };
    } // namespace realsense
} // namespace piplup