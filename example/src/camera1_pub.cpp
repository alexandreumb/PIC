#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <time.h>       // clock_gettime, CLOCK_MONOTONIC

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

// ── must match the subscriber setting ────────────────────────────────────────
// When USE_CLOCK_MONOTONIC = 1 in the subscriber, set it to 1 here too.
// Both sides must use the same clock or diffs will be meaningless.
#define USE_CLOCK_MONOTONIC   1

// ── globals for signal handling ──────────────────────────────────────────────
std::atomic<bool> stop{false};

void signal_handler(int) { stop = true; }

// ── helper: read CLOCK_MONOTONIC as nanoseconds ───────────────────────────────
inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// ── constants — must match camera and fixed_size_msgs layout ─────────────────
constexpr int      CAM_INDEX   = 2;
constexpr int      IMG_WIDTH   = 640;
constexpr int      IMG_HEIGHT  = 480;
constexpr int      IMG_TYPE    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr size_t   PIXEL_BYTES = 3;
constexpr uint32_t CAM_FREQ_HZ = 30;              // set to known camera frequency

int main(int argc, char ** argv)
{
    // ── signal setup ─────────────────────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── ROS2 init ────────────────────────────────────────────────────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    // ── QoS: must be best_effort + volatile for iceoryx zero-copy ────────────
    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

    auto pub = node->create_publisher<Image8Mb>("camera", qos);

    // ── camera init ──────────────────────────────────────────────────────────
    cv::VideoCapture cap(CAM_INDEX, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open camera index %d", CAM_INDEX);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  IMG_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // ── pre-compute sizes and validate against fixed message buffer ───────────
    const size_t step       = IMG_WIDTH * PIXEL_BYTES;
    const size_t frame_size = step * IMG_HEIGHT;

    {
        // Borrow once just to check pool chunk is large enough, then discard.
        // data is a fixed C array uint8[12582912] — use sizeof, not .size()
        auto probe = pub->borrow_loaned_message();
        constexpr size_t buf_size = sizeof(probe.get().data);
        if (frame_size > buf_size) {
            RCLCPP_ERROR(
                node->get_logger(),
                "Frame size %zu exceeds message buffer %zu. "
                "Check fixed_size_msgs definition.",
                frame_size, buf_size);
            return 1;
        }
    }  // probe is returned to pool here

    RCLCPP_INFO(node->get_logger(), "Camera publisher started — zero-copy path");

    // ── main capture + publish loop ───────────────────────────────────────────
    while (!stop && rclcpp::ok())
    {
        // ── Step 1: borrow a loaned chunk from iceoryx ────────────────────────
        auto loaned_msg = pub->borrow_loaned_message();
        auto & msg      = loaned_msg.get();

        // ── Step 2: construct a cv::Mat header over iceoryx shared memory ─────
        //    No allocation — OpenCV will write directly into the loaned buffer.
        //    Width, height and type must be known before read() is called.
        cv::Mat frame(
            IMG_HEIGHT,
            IMG_WIDTH,
            IMG_TYPE,
            msg.data.data()        // ← raw C array, no .data() needed
        );

        // ── Step 3: capture directly into iceoryx memory (zero-copy) ──────────
        if (!cap.read(frame)) {
            RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
            // loaned_msg destructor returns chunk to pool automatically
            continue;
        }

        // ── Step 4: fill metadata ─────────────────────────────────────────────
        // Timestamp sampled immediately after read() — closest to capture time.
        // Must use the same clock as the subscriber — both USE_CLOCK_MONOTONIC
        // must be set to the same value or latency diffs will be wrong.
#if USE_CLOCK_MONOTONIC
        msg.timestamp    = monotonic_now_ns();
#else
        msg.timestamp    = node->now().nanoseconds();
#endif
        msg.width        = IMG_WIDTH;
        msg.height       = IMG_HEIGHT;
        msg.step         = static_cast<uint32_t>(step);
        msg.is_bigendian = false;
        msg.frequency    = CAM_FREQ_HZ;

        // ── Step 5: publish — iceoryx transfers ownership, no copy ────────────
        // publish_timestamp sampled as late as possible — right before handoff
        // to iceoryx. The interval (publish_timestamp → subscriber receive_ns)
        // isolates pure transport + wakeup jitter from camera capture time.
#if USE_CLOCK_MONOTONIC
        msg.publish_timestamp = monotonic_now_ns();
#else
        msg.publish_timestamp = node->now().nanoseconds();
#endif
        pub->publish(std::move(loaned_msg));
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down camera publisher");
    cap.release();
    rclcpp::shutdown();
    return 0;
}