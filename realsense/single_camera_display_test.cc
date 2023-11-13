#include "realsense/realsensed400.h"
#include <opencv2/opencv.hpp>

int main(int argc, char * argv[])
{
    drake::systems::sensors::CameraConfig config;
    config.width = 640;
    config.height = 480;
    config.fps = 30;
    piplup::realsense::RealSenseD400 cam0("141722074426", config);
    piplup::realsense::RealSenseD400 cam1("938422072139", config);

    drake::systems::Simulator<double> simulator(cam0);
    simulator.set_target_realtime_rate(1.0);
    while(true)
    {
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0 / config.fps);
        auto depth_img = cam0.GetOutputPort("depth_image")
                             .Eval<drake::systems::sensors::ImageDepth16U>(
                                 cam0.GetMyContextFromRoot(simulator.get_context()));
        auto color_img = cam0.GetOutputPort("color_image")
                             .Eval<drake::systems::sensors::ImageRgba8U>(
                                 cam0.GetMyContextFromRoot(simulator.get_context()));
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