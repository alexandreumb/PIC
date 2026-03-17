#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_set.hpp"
#include <opencv4/opencv2/opencv.hpp>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <limits>
#include <time.h>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

// ── compile options ───────────────────────────────────────────────────────────
// USE_WAITSET 1 → manual WaitSet loop (lower latency, no executor overhead)
// USE_WAITSET 0 → standard rclcpp::spin with callback (simpler, easier to integrate)
#define USE_WAITSET         1
#define USE_CLOCK_MONOTONIC 1

std::atomic<bool> stop{false};
void signal_handler(int) { stop = true; }

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

constexpr int IMG_TYPE = CV_8UC3;

// ── latency tracking ──────────────────────────────────────────────────────────
struct LatencyStats {
    double   total_full_us      = 0.0;
    double   total_transport_us = 0.0;
    double   min_transport_us   = std::numeric_limits<double>::max();
    double   max_transport_us   = 0.0;
    uint64_t frame_count        = 0;
};

// ── placeholder processing pipeline ──────────────────────────────────────────
void process_image(const cv::Mat & img) { (void)img; }

// ── process a received message — shared by both WaitSet and callback paths ───
void handle_message(
    const Image8Mb & msg,
    LatencyStats & stats,
    rclcpp::Logger logger)
{
#if USE_CLOCK_MONOTONIC
    const int64_t receive_ns = monotonic_now_ns();
#else
    // NOTE: avoid node->now() here — pass clock if needed
    const int64_t receive_ns = monotonic_now_ns();
#endif

    const double full_us      = static_cast<double>(receive_ns - msg.timestamp)         / 1000.0;
    const double transport_us = static_cast<double>(receive_ns - msg.publish_timestamp) / 1000.0;

    stats.total_full_us      += full_us;
    stats.total_transport_us += transport_us;
    stats.min_transport_us    = std::min(stats.min_transport_us, transport_us);
    stats.max_transport_us    = std::max(stats.max_transport_us, transport_us);
    ++stats.frame_count;

    // zero-copy Mat header over received buffer — no memcpy
    cv::Mat img(
        static_cast<int>(msg.height),
        static_cast<int>(msg.width),
        IMG_TYPE,
        const_cast<uint8_t *>(msg.data.data())
    );

    process_image(img);

    if (stats.frame_count % 150 == 0) {
        RCLCPP_INFO(logger,
            "Frames: %lu | full A→D: %.1f us | transport B→D: %.1f us "
            "| min: %.1f us | max: %.1f us",
            stats.frame_count,
            stats.total_full_us      / static_cast<double>(stats.frame_count),
            stats.total_transport_us / static_cast<double>(stats.frame_count),
            stats.min_transport_us,
            stats.max_transport_us);
    }
}

void print_summary(const LatencyStats & stats, rclcpp::Logger logger)
{
    if (stats.frame_count == 0) {
        RCLCPP_WARN(logger, "No frames received");
        return;
    }
    const double avg_full      = stats.total_full_us      / static_cast<double>(stats.frame_count);
    const double avg_transport = stats.total_transport_us / static_cast<double>(stats.frame_count);
    RCLCPP_INFO(logger, "──────────────────────────────────────────");
    RCLCPP_INFO(logger, "Transport time summary");
    RCLCPP_INFO(logger, "  Frames received      : %lu",   stats.frame_count);
    RCLCPP_INFO(logger, "  Full latency   A→D   : %.1f us (%.3f ms)", avg_full,      avg_full      / 1000.0);
    RCLCPP_INFO(logger, "  Transport only B→D   : %.1f us (%.3f ms)", avg_transport, avg_transport / 1000.0);
    RCLCPP_INFO(logger, "  Camera buffering A→B : %.1f us (%.3f ms)",
        avg_full - avg_transport, (avg_full - avg_transport) / 1000.0);
    RCLCPP_INFO(logger, "  Transport min B→D    : %.1f us", stats.min_transport_us);
    RCLCPP_INFO(logger, "  Transport max B→D    : %.1f us", stats.max_transport_us);
    RCLCPP_INFO(logger, "──────────────────────────────────────────");
}

int main(int argc, char ** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_subscriber_node");

    // ── QoS: must match publisher exactly ────────────────────────────────────
    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

    LatencyStats stats;

#if USE_WAITSET
    // ─────────────────────────────────────────────────────────────────────────
    // WaitSet path — manual event loop, no executor overhead
    // take_loaned_message() gives zero-copy access to CycloneDDS shared memory
    // ─────────────────────────────────────────────────────────────────────────

    // dummy callback required by create_subscription API even with WaitSet
    auto sub = node->create_subscription<Image8Mb>(
        "camera", qos,
        [](const Image8Mb::SharedPtr) {}
    );

    // NOTE: sub->can_loan_messages() exists but take_loaned_message is not
    // exposed on rclcpp::Subscription in Humble — zero-copy receive requires
    // either Jazzy or the native iceoryx subscriber (camera_subscriber_native_iox).
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(sub);

    RCLCPP_INFO(node->get_logger(), "Subscriber started — WaitSet mode");

    while (!stop && rclcpp::ok())
    {
        // block up to 100ms — allows checking !stop periodically
        auto wait_result = wait_set.wait(std::chrono::milliseconds(100));

        if (wait_result.kind() != rclcpp::WaitResultKind::Ready) {
            continue;
        }

        // ── NOTE: take_loaned_message is not exposed on rclcpp::Subscription
        // in Humble — LoanedMessage only exists on the publisher side.
        // Standard take() with memcpy is used here.
        // On Jazzy this block can be replaced with sub->take_loaned_message()
        // for true zero-copy receive through rclcpp.
        // For zero-copy receive on Humble use camera_subscriber_native_iox.cpp
        // which bypasses rclcpp entirely using UntypedSubscriber directly.
        auto msg = std::make_unique<Image8Mb>();
        rclcpp::MessageInfo msg_info;
        if (sub->take(*msg, msg_info)) {
            handle_message(*msg, stats, node->get_logger());
        }
    }

#else
    // ─────────────────────────────────────────────────────────────────────────
    // Standard callback path — simpler, works with any executor
    // Uses SharedPtr — copy path, no zero-copy
    // Switch to take_loaned_message in WaitSet mode for true zero-copy
    // ─────────────────────────────────────────────────────────────────────────

    RCLCPP_INFO(node->get_logger(), "Subscriber started — callback mode");

    auto sub = node->create_subscription<Image8Mb>(
        "camera", qos,
        [&](const Image8Mb::SharedPtr msg) {
            handle_message(*msg, stats, node->get_logger());
        }
    );

    rclcpp::spin(node);

#endif

    print_summary(stats, node->get_logger());
    RCLCPP_INFO(node->get_logger(), "Shutting down");
    rclcpp::shutdown();
    return 0;
}