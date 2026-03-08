#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>

// ── native iceoryx headers ────────────────────────────────────────────────────
#include "iceoryx_posh/popo/untyped_subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <sys/mman.h>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

// ── service description — derived from rmw_iceoryx name conversion ────────────
// rule: {type_name, topic_name, "data"} for all ROS2 topics
// type_name  = package/msg/Type  → "fixed_size_msgs/msg/Image8Mb"
// topic_name = /camera
// event      = "data"            (always for ROS2)
static constexpr char IOX_SERVICE[]  = "fixed_size_msgs/msg/Image8Mb";
static constexpr char IOX_INSTANCE[] = "/camera";
static constexpr char IOX_EVENT[]    = "data";

// ── options ───────────────────────────────────────────────────────────────────
#define USE_CLOCK_MONOTONIC  1
#define USE_RT_SCHEDULING    1
#define USE_CPU_AFFINITY     1
#define USE_BUSY_WAIT        0   // 0 = WaitSet-like blocking, 1 = tight spin

constexpr int      SUBSCRIBER_CORE = 3;
constexpr int      RT_PRIORITY     = 80;
constexpr int      IMG_TYPE        = CV_8UC3;

// ── globals ───────────────────────────────────────────────────────────────────
std::atomic<bool> stop{false};

static double   total_latency_us     = 0.0;
static double   transport_latency_us = 0.0;
static uint64_t frame_count          = 0;

void signal_handler(int) { stop = true; }

// ── helpers ───────────────────────────────────────────────────────────────────
inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

void configure_thread()
{
#if USE_RT_SCHEDULING
    struct sched_param param{};
    param.sched_priority = RT_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "[WARN] Failed to set RT scheduling\n";
    } else {
        std::cout << "[INFO] RT scheduling set: SCHED_FIFO priority " << RT_PRIORITY << "\n";
    }
#endif
#if USE_CPU_AFFINITY
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(SUBSCRIBER_CORE, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
        std::cerr << "[WARN] Failed to set CPU affinity\n";
    } else {
        std::cout << "[INFO] Thread pinned to core " << SUBSCRIBER_CORE << "\n";
    }
#endif
}

// ── placeholder pipeline ──────────────────────────────────────────────────────
void process_image(const cv::Mat & img) { (void)img; }

int main(int argc, char ** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── ROS2 init — still needed for logging and result publishing ────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_subscriber_node");

    configure_thread();

    // ── native iceoryx runtime init ───────────────────────────────────────────
    // Must be called once per process. The runtime name must be unique.
    // This connects directly to RouDi shared memory — same RouDi that
    // rmw_iceoryx uses, so publisher and subscriber see the same segments.
    iox::runtime::PoshRuntime::initRuntime("camera_subscriber_native");

    // ── native untyped subscriber ─────────────────────────────────────────────
    // UntypedSubscriber gives us a raw void* into shared memory — zero copy.
    // The service description must match exactly what rmw_iceoryx registered:
    //   service  = type_name  = "fixed_size_msgs/msg/Image8Mb"
    //   instance = topic_name = "/camera"
    //   event    = "data"     (always for ROS2 topics in rmw_iceoryx)
    iox::popo::UntypedSubscriber iox_sub(
        {IOX_SERVICE, IOX_INSTANCE, IOX_EVENT},
        []() {
            iox::popo::SubscriberOptions opts;
            opts.queueCapacity = 1;           // keep only latest — drop stale frames
            opts.historyRequest = 0;
            return opts;
        }()
    );

    RCLCPP_INFO(node->get_logger(),
        "Native iceoryx subscriber started — service: %s / %s / %s",
        IOX_SERVICE, IOX_INSTANCE, IOX_EVENT);

    mlockall(MCL_CURRENT | MCL_FUTURE);
    // ── main loop ─────────────────────────────────────────────────────────────
    while (!stop && rclcpp::ok())
    {
#if USE_BUSY_WAIT
        // tight spin — zero wakeup latency, burns one CPU core
        iox_sub.take()
        .and_then([&](const void * userPayload) {

#else
        // blocking wait — yields CPU until data arrives
        // iceoryx WaitSet equivalent for untyped subscriber
        iox_sub.take()
        .and_then([&](const void * userPayload) {

#endif
            // ── Step 1: timestamp receive — userPayload is direct shm pointer ─
#if USE_CLOCK_MONOTONIC
            const int64_t receive_ns = monotonic_now_ns();
#else
            const int64_t receive_ns = node->now().nanoseconds();
#endif
            // ── Step 2: cast to message type — zero copy, no memcpy ───────────
            // userPayload points directly into iceoryx shared memory segment.
            // The publisher wrote Image8Mb in-place via borrow_loaned_message.
            const auto * msg = static_cast<const Image8Mb *>(userPayload);

            // ── Step 3: measure latency ───────────────────────────────────────
            const double full_us = static_cast<double>(
                receive_ns - msg->timestamp) / 1000.0;
            const double transport_us = static_cast<double>(
                receive_ns - msg->publish_timestamp) / 1000.0;

            total_latency_us     += full_us;
            transport_latency_us += transport_us;
            ++frame_count;

            // ── Step 4: construct Mat header over shared memory ───────────────
            // NO memcpy — Mat points directly at iceoryx chunk.
            // process_image() must complete before release() is called below.
            cv::Mat img(
                static_cast<int>(msg->height),
                static_cast<int>(msg->width),
                IMG_TYPE,
                const_cast<uint8_t *>(msg->data.data())
            );

            // ── Step 5: process ───────────────────────────────────────────────
            process_image(img);

            // ── Step 6: release chunk back to iceoryx pool ────────────────────
            // Must be called — otherwise the pool exhausts and publisher stalls.
            iox_sub.release(userPayload);

            if (frame_count % 150 == 0) {  // log every ~5s at 30fps
                RCLCPP_INFO(node->get_logger(),
                    "Frames: %lu | full A→D: %.1f us | transport B→D: %.1f us",
                    frame_count,
                    total_latency_us     / static_cast<double>(frame_count),
                    transport_latency_us / static_cast<double>(frame_count));
            }
        })
        .or_else([](auto &) {
            // no chunk available — yield to avoid starving other threads
            __asm__ volatile("pause" ::: "memory");  // CPU hint to reduce power/contention
        });
    }

    // ── shutdown summary ──────────────────────────────────────────────────────
    if (frame_count > 0) {
        const double avg_full_us      = total_latency_us     / static_cast<double>(frame_count);
        const double avg_transport_us = transport_latency_us / static_cast<double>(frame_count);
        RCLCPP_INFO(node->get_logger(),
            "──────────────────────────────────────────");
        RCLCPP_INFO(node->get_logger(), "Transport time summary");
        RCLCPP_INFO(node->get_logger(),
            "  Frames received      : %lu", frame_count);
        RCLCPP_INFO(node->get_logger(),
            "  Full latency   A→D   : %.1f us (%.3f ms)",
            avg_full_us, avg_full_us / 1000.0);
        RCLCPP_INFO(node->get_logger(),
            "  Transport only B→D   : %.1f us (%.3f ms)",
            avg_transport_us, avg_transport_us / 1000.0);
        RCLCPP_INFO(node->get_logger(),
            "  Camera buffering A→B : %.1f us (%.3f ms)",
            avg_full_us - avg_transport_us,
            (avg_full_us - avg_transport_us) / 1000.0);
        RCLCPP_INFO(node->get_logger(),
            "──────────────────────────────────────────");
    } else {
        RCLCPP_WARN(node->get_logger(), "No frames received — no latency data");
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down native iceoryx subscriber");
    rclcpp::shutdown();
    return 0;
}