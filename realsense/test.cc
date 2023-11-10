#include <drake/common/drake_copyable.h>
#include <drake/common/name_value.h>
#include <drake/common/scoped_singleton.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/sensors/camera_config.h>
#include <drake/systems/sensors/image.h>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

namespace piplup
{
    namespace realsense
    {
        using namespace drake;

        struct RealsenseInterfaceConfig
        {
            std::string serial_number;

            template<typename Archive>
            void Serialize(Archive * a)
            {
                a->Visit(DRAKE_NVP(serial_number));
            }
        };

        class RealSenseD400 : public systems::LeafSystem<double>
        {
        public:
            RealSenseD400(std::string device_serial_number,
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
                    if(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                       == device_serial_number)
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
            };

            void PollForImages(const systems::Context<double> & context,
                               systems::State<double> * state) const
            {
                rs2::frameset frames;
                if(pipeline_.poll_for_frames(&frames))
                {
                    drake::log()->info("Test");
                    rs2::frame color_frame = frames.get_color_frame();
                    rs2::depth_frame depth_frame = frames.get_depth_frame();
                    auto & color_img_state =
                        state->get_mutable_abstract_state<systems::sensors::ImageRgba8U>(
                            color_state_idx_);
                    auto & depth_img_state =
                        state
                            ->get_mutable_abstract_state<systems::sensors::ImageDepth16U>(
                                depth_state_idx_);
                    color_img_state.resize(color_width_, color_height_);
                    depth_img_state.resize(depth_width_, depth_height_);
                    memcpy(depth_img_state.at(0, 0),
                           depth_frame.get_data(),
                           depth_img_state.size());
                    memcpy(color_img_state.at(0, 0),
                           color_frame.get_data(),
                           color_img_state.size());
                }
            }

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

int main(int argc, char * argv[])
{
    drake::systems::sensors::CameraConfig config;
    config.width = 640;
    config.height = 480;
    config.fps = 30;
    piplup::realsense::RealSenseD400 sys("141722074426", config);
    drake::systems::Simulator<double> simulator(sys);
    simulator.set_target_realtime_rate(1.0);
    while(true)
    {
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0 / config.fps);
        auto depth_img = sys.GetOutputPort("depth_image")
                             .Eval<drake::systems::sensors::ImageDepth16U>(
                                 sys.GetMyContextFromRoot(simulator.get_context()));
        auto color_img = sys.GetOutputPort("color_image")
                             .Eval<drake::systems::sensors::ImageRgba8U>(
                                 sys.GetMyContextFromRoot(simulator.get_context()));
        // std::cout << depth_img.height() << "," << depth_img.width() << "\n";
        if(depth_img.height() != 0)
        {
            cv::Mat depth(cv::Size(640, 480),
                          CV_16U,
                          (void *)depth_img.at(0, 0),
                          cv::Mat::AUTO_STEP);
            cv::Mat color(cv::Size(640, 480),
                          CV_8UC4,
                          (void *)color_img.at(0, 0),
                          cv::Mat::AUTO_STEP);
            double min;
            double max;
            cv::minMaxIdx(depth, &min, &max);
            cv::convertScaleAbs(depth, depth, 255 / max);
            cv::cvtColor(color, color, cv::COLOR_RGBA2BGR);
            cv::imshow("Display Depth", depth);

            cv::imshow("Display Image", color);
            if(cv::waitKey(30) >= 0)
                break;
        }
    }
}