#include "realsense/realsensed400.h"

namespace piplup
{
    namespace realsense
    {
        using namespace drake;
        RealSenseD400::RealSenseD400(std::string device_serial_number,
                                     const systems::sensors::CameraConfig & camera_config)
          : context_(drake::GetScopedSingleton<rs2::context>())
          , pipeline_(*context_)
          , depth_width_(camera_config.width)
          , depth_height_(camera_config.height)
          , color_width_(camera_config.width)
          , color_height_(camera_config.height)
        {
            double polling_rate = 0.001;

            color_state_idx_ =
                this->DeclareAbstractState(Value<systems::sensors::ImageRgba8U>());
            depth_state_idx_ =
                this->DeclareAbstractState(Value<systems::sensors::ImageDepth16U>());
            this->DeclareStateOutputPort("color_image", color_state_idx_);
            this->DeclareStateOutputPort("depth_image", depth_state_idx_);
            this->DeclarePeriodicUnrestrictedUpdateEvent(
                polling_rate, 0.0, &RealSenseD400::PollForImages);

            // Check if device exists
            bool found_device = false;
            for(auto && device : context_->query_devices())
            {
                drake::log()->info(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
                if(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == device_serial_number)
                {
                    found_device = true;
                    break;
                }
            }
            if(!found_device)
            {
                throw std::runtime_error(
                    fmt::format("Failed to find Realsense D400 with serial number {}",
                                device_serial_number));
            }

            rs2::config cfg;
            cfg.enable_device(device_serial_number);
            cfg.enable_stream(RS2_STREAM_COLOR,
                              color_width_,
                              color_height_,
                              RS2_FORMAT_RGBA8,
                              camera_config.fps);
            cfg.enable_stream(RS2_STREAM_DEPTH,
                              depth_width_,
                              depth_height_,
                              RS2_FORMAT_Z16,
                              camera_config.fps);
            pipeline_.start(cfg);
        }
        void RealSenseD400::PollForImages(const systems::Context<double> & context,
                                          systems::State<double> * state) const
        {
            rs2::frameset frames;
            if(pipeline_.poll_for_frames(&frames))
            {
                drake::log()->info("Received Frames");
                rs2::frame color_frame = frames.get_color_frame();
                rs2::depth_frame depth_frame = frames.get_depth_frame();
                auto & color_img_state =
                    state->get_mutable_abstract_state<systems::sensors::ImageRgba8U>(
                        color_state_idx_);
                auto & depth_img_state =
                    state->get_mutable_abstract_state<systems::sensors::ImageDepth16U>(
                        depth_state_idx_);
                color_img_state.resize(color_width_, color_height_);
                depth_img_state.resize(depth_width_, depth_height_);
                memcpy(
                    depth_img_state.at(0, 0),
                    depth_frame.get_data(),
                    depth_img_state.size() * 2); // TODO why do we have to multiply by 2
                memcpy(color_img_state.at(0, 0),
                       color_frame.get_data(),
                       color_img_state.size());
            }
        }
    } // namespace realsense
} // namespace piplup