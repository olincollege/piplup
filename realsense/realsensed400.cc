#include "realsense/realsensed400.h"

namespace piplup
{
    namespace realsense
    {
        using namespace drake;
        RealSenseD400::RealSenseD400(
            std::string device_serial_number,
            const systems::sensors::CameraConfig & camera_config,
            const multibody::MultibodyPlant<double> & multibody_plant)
          : context_(drake::GetScopedSingleton<rs2::context>())
          , pipeline_(*context_)
          , depth_width_(camera_config.width)
          , depth_height_(camera_config.height)
          , color_width_(camera_config.width)
          , color_height_(camera_config.height)
          , multibody_plant_(multibody_plant)
          , camera_base_frame_(camera_config.X_PB.base_frame.value())
        {
            double polling_rate = 0.001;
            color_state_idx_ =
                this->DeclareAbstractState(Value<systems::sensors::ImageRgba8U>());
            depth_state_idx_ =
                this->DeclareAbstractState(Value<systems::sensors::ImageDepth16U>());

            this->DeclareStateOutputPort("color_image", color_state_idx_);
            this->DeclareStateOutputPort("depth_image_16u", depth_state_idx_);
            this->DeclareAbstractOutputPort("body_pose_in_world",
                                            &RealSenseD400::CalcX_WB);

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
            // cfg.enable_stream(RS2_STREAM_COLOR,
            //                   640,
            //                   480,
            //                   RS2_FORMAT_RGBA8,
            //                   camera_config.fps);
            // cfg.enable_stream(RS2_STREAM_DEPTH,
            //                   depth_width_,
            //                   depth_height_,
            //                   RS2_FORMAT_Z16,
            //                   camera_config.fps);

            rs2::pipeline_profile selection = pipeline_.start(cfg);
            auto depth_stream =
                selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            auto i = depth_stream.get_intrinsics();
            depth_camera_info_ = std::make_unique<systems::sensors::CameraInfo>(
                depth_stream.width(), depth_stream.height(), i.fx, i.fy, i.ppx, i.ppy);
            std::cout << "w:" << depth_stream.width() << "\n";
            std::cout << "h:" << depth_stream.height() << "\n";
            std::cout << depth_stream.fps() << "\n";
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
                std::cout << "hi\n";
                memcpy(
                    depth_img_state.at(0, 0),
                    depth_frame.get_data(),
                    depth_img_state.size() * 2); // TODO why do we have to multiply by 2
                memcpy(color_img_state.at(0, 0),
                       color_frame.get_data(),
                       color_img_state.size());
            }
        }

        const systems::sensors::CameraInfo & RealSenseD400::depth_camera_info() const
        {
            return *depth_camera_info_;
        }

        void RealSenseD400::CalcX_WB(const systems::Context<double> & context,
                                     math::RigidTransformd * output) const
        {
            output = &multibody_plant_.CalcRelativeTransform(
                context,
                multibody_plant_.world_frame(),
                multibody_plant_.GetFrameByName(camera_base_frame_));
        }

    } // namespace realsense
} // namespace piplup