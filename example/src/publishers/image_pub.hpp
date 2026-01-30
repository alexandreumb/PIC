#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_hoofs/posix_wrapper/signal_watcher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iceoryx_posh/capro/service_description.hpp"
#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"
#include "librealsense2/hpp/rs_frame.hpp"
#include <chrono>
#include <thread>
#include <opencv4/opencv2/opencv.hpp>

template <class MsgT>

class image_pub
{
public:
    image_pub(const iox::capro::ServiceDescription& description, bool robot)
        : robot_(robot),
        pub_(description)
    {
        if (robot_)
        {
            cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
            p_.start(cfg_);
        }
        else
        {
            img_ = cv::imread("/home/alexandre/large_image.jpg", cv::IMREAD_COLOR);
            frame_size_ = static_cast<size_t>(img_.step[0] * img_.rows);
        }
    }

    virtual ~image_pub()
    {
        std::cerr << "end" << std::endl;
    }

    void send_image()
    {
        if (robot_)
        {
            rs2::frameset frames = p_.wait_for_frames();
            rs2::depth_frame depth = frames.get_depth_frame();

            img_ = cv::Mat(
                cv::Size(depth.get_width(), depth.get_height()),
                CV_16UC1,
                (void *)depth.get_data(),
                cv::Mat::AUTO_STEP);

            frame_size_ = static_cast<size_t>(img_.step[0] * img_.rows);
        }

        pub_.loan()
            .and_then([&](auto &sample)
                      {
                sample.get()->is_bigendian = false;
                sample.get()->step = static_cast<uint32_t>(img_.cols * img_.elemSize());
                sample.get()->width = img_.cols;
                sample.get()->height = img_.rows;

                if (frame_size_ > sizeof(sample.get()->data)) {
                    std::cerr << "Incompatible sizes: frame " << frame_size_
                            << " > buffer " << sizeof(sample.get()->data) << std::endl;
                    return;
                }

                std::memcpy(sample.get()->data.data(), img_.data, frame_size_);

                sample.get()->timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                
                pub_.publish(std::move(sample)); })
            .or_else([](auto &error)
                     { std::cerr << "Unable to loan sample: " << static_cast<int>(error) << std::endl; });
    }

private:
    bool robot_;
    cv::Mat img_;
    rs2::pipeline p_;
    rs2::config cfg_;
    iox::popo::Publisher<MsgT> pub_;
    size_t frame_size_;
};
