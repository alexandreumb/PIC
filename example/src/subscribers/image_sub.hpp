#include "iceoryx_hoofs/posix_wrapper/signal_watcher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"

#include <chrono>
#include <thread>
#include <opencv4/opencv2/opencv.hpp>

template <class MsgT>

class image_sub
{
public:
    image_sub(const iox::capro::ServiceDescription &description, bool robot)
        : sub_(description),
          robot_(robot)
    {
        image_buffer_.resize(12582912); // 2048 * 2048 * 3
    }

    virtual ~image_sub()
    {
        std::cerr << "Average time: " << (average_round_time_ / k) << " for " << k << std::endl;
    }

    void receive_image()
    {
        sub_.take()
            .and_then([&](auto &sample)
                      {
                          const auto &msg_ = *sample;

                          auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::high_resolution_clock::now().time_since_epoch())
                                         .count();

                          auto msg_timestamp = msg_.timestamp;
                          auto diff = now - msg_timestamp;
                          average_round_time_ += diff;
                          ++k;  

                          cv::Mat image_(msg_.height, msg_.width, CV_16UC1, (void *)msg_.data.data()); 
                          cv::imwrite(std::string("/home/alexandre/image_subscriber") + ".jpg", image_);
                          
                        })
            .or_else([](auto &error)
                     { std::cerr << "Unable to loan sample: " << static_cast<int>(error) << std::endl; });
    }

private:
    iox::popo::Subscriber<MsgT> sub_;
    size_t frame_size_;
    int64_t average_round_time_;
    int64_t k;
    bool robot_;
    std::vector<uint8_t> image_buffer_;
};
