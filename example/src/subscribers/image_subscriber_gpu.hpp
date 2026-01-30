#include "iceoryx_posh/popo/subscriber.hpp"

#include <chrono>
#include <thread>
#include <opencv4/opencv2/opencv.hpp>

template<class MsgT>

class image_subscriber_gpu
{
public:
    image_subscriber_gpu(const iox::capro::ServiceDescription& description)
    : sub_(description)
    {
    }

    virtual ~image_subscriber_gpu() 
    {
        std::cerr << "end" << std::endl;
    }

    void receive_image()
    {
        sub_.take()
            .and_then([&](auto& sample) {
                const auto& msg_ = *sample;

                auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                
                auto msg_timestamp = msg_.timestamp;
                auto diff = now - msg_timestamp;

                cv::Mat image(msg_.height, msg_.width, CV_16UC1, (void*)msg_.data.data());
            })
            .or_else([](auto& error) {
                std::cerr << "Unable to loan sample: " << static_cast<int>(error) << std::endl;
            });
    }


private:
    cv::Mat img_;
    iox::popo::Subscriber<MsgT> sub_;
    size_t frame_size_;
};

