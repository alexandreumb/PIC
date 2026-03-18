#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <time.h>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

#define USE_CLOCK_MONOTONIC 1

std::atomic<bool> stop{false};
void signal_handler(int) { stop = true; }

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

constexpr int      CAM_INDEX   = 2;
constexpr int      IMG_WIDTH   = 640;
constexpr int      IMG_HEIGHT  = 480;
constexpr int      IMG_TYPE    = CV_8UC3;
constexpr size_t   PIXEL_BYTES = 3;
constexpr uint32_t CAM_FREQ_HZ = 30;

int main(int argc, char ** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    // ── QoS: reliable works with FastDDS, no special constraints ─────────────
    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

    auto pub = node->create_publisher<Image8Mb>("camera", qos);

    cv::VideoCapture cap(CAM_INDEX, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open camera index %d", CAM_INDEX);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  IMG_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    cap.set(cv::CAP_PROP_BUFFERSIZE,   1);

    const size_t step = IMG_WIDTH * PIXEL_BYTES;

    // ── allocate message once on heap — 12MB, never on stack ─────────────────
    auto msg = std::make_unique<Image8Mb>();

    // construct Mat header over msg buffer — cap.read() writes directly here
    cv::Mat frame(IMG_HEIGHT, IMG_WIDTH, IMG_TYPE, msg->data.data());

    RCLCPP_INFO(node->get_logger(), "FastDDS publisher started");

    while (!stop && rclcpp::ok())
    {
        
        if (!cap.read(frame)) {
            RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
            continue;
        }
        
#if USE_CLOCK_MONOTONIC
        msg->timestamp = monotonic_now_ns();
#else
        msg->timestamp = node->now().nanoseconds();
#endif

        msg->width        = IMG_WIDTH;
        msg->height       = IMG_HEIGHT;
        msg->step         = static_cast<uint32_t>(step);
        msg->is_bigendian = false;
        msg->frequency    = CAM_FREQ_HZ;

#if USE_CLOCK_MONOTONIC
        msg->publish_timestamp = monotonic_now_ns();
#else
        msg->publish_timestamp = node->now().nanoseconds();
#endif
        // ── FastDDS will memcpy into its shared memory segment internally ─────
        // No zero-copy API available on publisher side in Humble FastDDS.
        // FastDDS shm transport copies once into shm, subscriber copies once out.
        pub->publish(*msg);
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down FastDDS publisher");
    cap.release();
    rclcpp::shutdown();
    return 0;
}
