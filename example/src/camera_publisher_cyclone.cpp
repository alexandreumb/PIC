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

    // ── QoS: best_effort + volatile required for zero-copy loaned messages ────
    // CycloneDDS shared memory works with both reliable and best_effort,
    // but loaned messages require volatile durability.
    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

    auto pub = node->create_publisher<Image8Mb>("camera", qos);

    // ── verify loaned message support is active ───────────────────────────────
    // With CycloneDDS + shared memory this should return true.
    // If false, CycloneDDS fell back to UDP (RouDi not running or XML config missing).
    if (!pub->can_loan_messages()) {
        RCLCPP_WARN(node->get_logger(),
            "Loaned messages NOT supported — running without zero-copy. "
            "Check RouDi is running and CYCLONEDDS_URI is set correctly.");
    } else {
        RCLCPP_INFO(node->get_logger(), "Loaned messages supported — zero-copy active");
    }

    cv::VideoCapture cap(CAM_INDEX, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open camera index %d", CAM_INDEX);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  IMG_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    cap.set(cv::CAP_PROP_BUFFERSIZE,   1);

    const size_t step       = IMG_WIDTH * PIXEL_BYTES;
    const size_t frame_size = step * IMG_HEIGHT;

    // ── validate buffer size ──────────────────────────────────────────────────
    if (pub->can_loan_messages()) {
        auto probe = pub->borrow_loaned_message();
        constexpr size_t buf_size = sizeof(probe.get().data);
        if (frame_size > buf_size) {
            RCLCPP_ERROR(node->get_logger(),
                "Frame size %zu exceeds message buffer %zu.", frame_size, buf_size);
            return 1;
        }
    }

    RCLCPP_INFO(node->get_logger(), "Camera publisher started");

    while (!stop && rclcpp::ok())
    {
        if (pub->can_loan_messages())
        {
            // ── zero-copy path: capture directly into shared memory ───────────
            auto loaned_msg = pub->borrow_loaned_message();
            auto & msg      = loaned_msg.get();

            cv::Mat frame(IMG_HEIGHT, IMG_WIDTH, IMG_TYPE, msg.data.data());

            if (!cap.read(frame)) {
                RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
                continue;
            }

#if USE_CLOCK_MONOTONIC
            msg.timestamp = monotonic_now_ns();
#else
            msg.timestamp = node->now().nanoseconds();
#endif
            msg.width        = IMG_WIDTH;
            msg.height       = IMG_HEIGHT;
            msg.step         = static_cast<uint32_t>(step);
            msg.is_bigendian = false;
            msg.frequency    = CAM_FREQ_HZ;

#if USE_CLOCK_MONOTONIC
            msg.publish_timestamp = monotonic_now_ns();
#else
            msg.publish_timestamp = node->now().nanoseconds();
#endif
            pub->publish(std::move(loaned_msg));
        }
        else
        {
            auto msg_ptr = std::make_unique<Image8Mb>();
            Image8Mb & msg = *msg_ptr;
            cv::Mat frame(IMG_HEIGHT, IMG_WIDTH, IMG_TYPE, msg.data.data());

            if (!cap.read(frame)) {
                RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
                continue;
            }

#if USE_CLOCK_MONOTONIC
            msg.timestamp = monotonic_now_ns();
#else
            msg.timestamp = node->now().nanoseconds();
#endif
            msg.width        = IMG_WIDTH;
            msg.height       = IMG_HEIGHT;
            msg.step         = static_cast<uint32_t>(step);
            msg.is_bigendian = false;
            msg.frequency    = CAM_FREQ_HZ;

#if USE_CLOCK_MONOTONIC
            msg.publish_timestamp = monotonic_now_ns();
#else
            msg.publish_timestamp = node->now().nanoseconds();
#endif
            pub->publish(msg);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down camera publisher");
    cap.release();
    rclcpp::shutdown();
    return 0;
}