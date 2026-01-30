#include <thread>
#include <mutex>
#include <opencv4/opencv2/opencv.hpp>
#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iceoryx_hoofs/posix_wrapper/signal_watcher.hpp"

std::mutex frame_mutex;
cv::Mat latest_color, latest_depth, image;
int64_t k = 0;
int64_t average_round_time_;

void processingThread()
{
    while (!iox::posix::hasTerminationRequested())
    {
        cv::Mat color_copy, depth_copy, image_copy;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (latest_color.empty() && latest_depth.empty() && image.empty())
                continue;
            color_copy = latest_color.clone();
            depth_copy = latest_depth.clone();
            image_copy = image.clone();
        }

        // Do your image analysis here
        if (!color_copy.empty())
        {
            // e.g., detect objects, feature extraction
        }
        if (!depth_copy.empty())
        {
            // e.g., compute distance metrics
        }
        if (!image_copy.empty())
        {
            k++;
            cv::imwrite("/home/alexandre/image_subscriber.jpg", image_copy);
        }

        std::cout << "Average time " << (average_round_time_ / k) / 1e6 << " ms for " << k << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // adjust frequency
    }
}

int main()
{
    bool robot = false;

    iox::runtime::PoshRuntime::initRuntime("camera_sub");

    iox::popo::Subscriber<fixed_size_msgs::msg::Image8Mb> sub({"camera", "Image", "8Mb"});
    iox::popo::Subscriber<fixed_size_msgs::msg::Image8Mb> color_sub({"camera", "Image", "Color"});
    iox::popo::Subscriber<fixed_size_msgs::msg::Image8Mb> depth_sub({"camera", "Image", "Depth"});

    // Launch processing thread
    std::thread analysis_thread(processingThread);

    while (!iox::posix::hasTerminationRequested())
    {
        if (robot)
        {
            // Color
            color_sub.take().and_then([](auto &sample)
                                      {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_color = cv::Mat(sample->height, sample->width, CV_8UC3,
                                       (void*)sample->data.data(), sample->step).clone(); });

            // Depth
            depth_sub.take().and_then([](auto &sample)
                                      {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_depth = cv::Mat(sample->height, sample->width, CV_16UC1,
                                       (void*)sample->data.data(), sample->step).clone(); });

            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // avoid busy loop
        }
        else
        {
            sub.take().and_then([](auto &sample)
                                {
                std::lock_guard<std::mutex> lock(frame_mutex);
                image = cv::Mat(sample->height, sample->width, CV_8UC3,
                                       (void*)sample->data.data(), sample->step).clone();
                                       
                auto msg_timestamp = sample->timestamp;
                auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count(); 
                auto diff = now - msg_timestamp;
                average_round_time_ += diff; });

            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // avoid busy loop
        }
    }

    analysis_thread.join();
}
