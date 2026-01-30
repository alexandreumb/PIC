#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iceoryx_hoofs/posix_wrapper/signal_watcher.hpp"
#include "librealsense2/rs.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <iostream>

int main(int argc, char **argv)
{
    iox::runtime::PoshRuntime::initRuntime("camera_pub");

    bool robot_ = false; // true -> use actual camera; false -> static image
    cv::Mat color_img_, depth_img_;
    size_t color_size_ = 0, depth_size_ = 0;
    int32_t frequency = 100;
    std::unique_ptr<rclcpp::Rate> rate = std::make_unique<rclcpp::Rate>(frequency);
    int i = 1;

    rs2::pipeline p_;
    rs2::config cfg_;
    cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    iox::popo::Publisher<fixed_size_msgs::msg::Image8Mb> pub({"camera", "Image", "8Mb"});
    iox::popo::Publisher<fixed_size_msgs::msg::Image8Mb> color_pub({"camera", "Image", "Color"});
    iox::popo::Publisher<fixed_size_msgs::msg::Image8Mb> depth_pub({"camera", "Image", "Depth"});

    if (robot_)
    {
        p_.start(cfg_);
    }
    else
    {
        color_img_ = cv::imread("/home/alexandre/large_image.jpg", cv::IMREAD_COLOR);
        color_size_ = static_cast<size_t>(color_img_.step[0] * color_img_.rows);
    }

    while (!iox::posix::hasTerminationRequested())
    {
        if (robot_)
        {
            rs2::frameset frames;
            if (p_.poll_for_frames(&frames))
            {
                rs2::frame color_frame = frames.get_color_frame();
                rs2::frame depth_frame = frames.get_depth_frame();

                // Convert RealSense frames to cv::Mat
                color_img_ = cv::Mat(
                    cv::Size(color_frame.as<rs2::video_frame>().get_width(),
                             color_frame.as<rs2::video_frame>().get_height()),
                    CV_8UC3,
                    (void *)color_frame.get_data(),
                    cv::Mat::AUTO_STEP);

                depth_img_ = cv::Mat(
                    cv::Size(depth_frame.as<rs2::video_frame>().get_width(),
                             depth_frame.as<rs2::video_frame>().get_height()),
                    CV_16UC1,
                    (void *)depth_frame.get_data(),
                    cv::Mat::AUTO_STEP);

                color_size_ = static_cast<size_t>(color_img_.step[0] * color_img_.rows);
                depth_size_ = static_cast<size_t>(depth_img_.step[0] * depth_img_.rows);

                // Publish both streams
                auto publish_image = [&](auto &pub, const cv::Mat &img, size_t frame_size)
                {
                    pub.loan()
                        .and_then([&](auto &sample)
                                  {
                                      sample.get()->is_bigendian = false;
                                      sample.get()->step = static_cast<uint32_t>(img.cols * img.elemSize());
                                      sample.get()->width = img.cols;
                                      sample.get()->height = img.rows;

                                      if (frame_size > sizeof(sample.get()->data))
                                      {
                                          std::cerr << "Incompatible sizes: frame "
                                                    << frame_size << " > buffer "
                                                    << sizeof(sample.get()->data) << std::endl;
                                          return;
                                      }

                                      std::memcpy(sample.get()->data.data(), img.data, frame_size);

                                      sample.get()->timestamp =
                                          std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::high_resolution_clock::now().time_since_epoch())
                                              .count();

                                      pub.publish(std::move(sample)); })
                        .or_else([](auto &error)
                                 { std::cerr << "Unable to loan sample: "
                                             << static_cast<int>(error) << std::endl; });
                };

                publish_image(color_pub, color_img_, color_size_);
                publish_image(depth_pub, depth_img_, depth_size_);
            }
        }
        else
        {   
            pub.loan()
                .and_then([&](auto &sample)
                          {
                              sample.get()->is_bigendian = false;
                              sample.get()->step = static_cast<uint32_t>(color_img_.cols * color_img_.elemSize());
                              sample.get()->width = color_img_.cols;
                              sample.get()->height = color_img_.rows;
                              sample.get()->frequency = frequency;

                              std::memcpy(sample.get()->data.data(), color_img_.data, color_size_);
                              sample.get()->timestamp =
                                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::high_resolution_clock::now().time_since_epoch())
                                      .count();
                              pub.publish(std::move(sample)); });
            
            rate->sleep();
        }
    }

    return 0;
}
