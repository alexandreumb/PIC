#include "iceoryx_posh/popo/publisher.hpp"
#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"
#include "librealsense2/hpp/rs_frame.hpp"
#include <chrono>
#include <thread>
#include <opencv4/opencv2/opencv.hpp>

template<class MsgT>

class image_publisher_gpu
{
public:
    image_publisher_gpu(const iox::capro::ServiceDescription& description)
    : pub_(description)
    {
        //cfg_.enable_stream();
        //cfg_.enable_device();
        p_.start(cfg_);
    }

    virtual ~image_publisher_gpu() 
    {
        std::cerr << "end" << std::endl;
    }

    void send_image()
    {
        rs2::frameset frames = p_.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();

        img_ = cv::Mat(
            cv::Size(depth.get_width(), depth.get_height()),
            CV_16UC1,
            (void*)depth.get_data(),
            cv::Mat::AUTO_STEP
        );

        /* IF ANY PROCESSING IS NEEDED
        cv::cuda::GPUMat gpu_img;
        gpu_img.upload(img_);
        
        gpu_img.download(img_);
        */

        const size_t frame_size = static_cast<size_t>(img_.step[0] * img_.rows);

        pub_.loan()
            .and_then([&](auto& sample) {

                sample.get()->is_bigendian = false;
                sample.get()->step = static_cast<uint32_t>(img_.cols * img_.elemSize());
                sample.get()->width = img_.cols;
                sample.get()->height = img_.rows;

                if (frame_size > sizeof(sample.get()->data)) {
                    std::cerr << "Incompatible sizes: frame " << frame_size
                            << " > buffer " << sizeof(sample.get()->data) << std::endl;
                    return;
                }

                std::memcpy(sample.get()->data.data(), img_.data, frame_size);

                sample.get()->timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();

                pub_.publish(std::move(sample));
            })
            .or_else([](auto& error) {
                std::cerr << "Unable to loan sample: " << static_cast<int>(error) << std::endl;
            });
    }


private:
    cv::Mat img_;
    rs2::pipeline p_;
    rs2::config cfg_;
    iox::popo::Publisher<MsgT> pub_;
    size_t frame_size_;
};

