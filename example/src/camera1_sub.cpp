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
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <limits>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

// ── service description — derived from rmw_iceoryx name conversion ────────────
// rule: {type_name, topic_name, "data"} for all ROS2 topics
// type_name  = package/msg/Type  → "fixed_size_msgs/msg/Image8Mb"
// topic_name = /camera
// event      = "data"            (always for ROS2)
static constexpr char IOX_SERVICE[]  = "fixed_size_msgs/msg/Image8Mb";
static constexpr char IOX_INSTANCE[] = "/camera";
static constexpr char IOX_EVENT[]    = "data";
static const std::string OUTPUT_DIR = std::string(getenv("HOME")) + "/latency_data";

// ── options ───────────────────────────────────────────────────────────────────
#define USE_CLOCK_MONOTONIC  1
#define USE_RT_SCHEDULING    0
#define USE_CPU_AFFINITY     0
#define USE_BUSY_WAIT        1   // 0 = WaitSet-like blocking, 1 = tight spin

constexpr int      SUBSCRIBER_CORE = 3;
constexpr int      RT_PRIORITY     = 80;
constexpr int      IMG_TYPE        = CV_8UC3;

// ── globals ───────────────────────────────────────────────────────────────────
std::atomic<bool> stop{false};

static double   total_full_us     = 0.0;
static double   total_transport_us = 0.0;

void signal_handler(int) { stop = true; }

// ── helpers ───────────────────────────────────────────────────────────────────
inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

std::string make_csv_path()
{
    system(("mkdir -p " + OUTPUT_DIR).c_str());
    time_t now = time(nullptr);
    struct tm * t = localtime(&now);
    std::ostringstream ss;
    ss << OUTPUT_DIR << "/latency_run_"
       << std::setfill('0')
       << (t->tm_year + 1900) << "-"
       << std::setw(2) << (t->tm_mon + 1) << "-"
       << std::setw(2) << t->tm_mday << "_"
       << std::setw(2) << t->tm_hour << "-"
       << std::setw(2) << t->tm_min  << "-"
       << std::setw(2) << t->tm_sec
       << ".csv";
    return ss.str();
}

void save_csv(
    const std::string & path,
    const std::vector<double> & full_us,
    const std::vector<double> & transport_us)
{
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[ERROR] Cannot open " << path << " for writing\n";
        return;
    }
    f << "frame,full_us,transport_us\n";
    for (size_t i = 0; i < full_us.size(); ++i) {
        f << i << ","
          << std::fixed << std::setprecision(2) << full_us[i] << ","
          << std::fixed << std::setprecision(2) << transport_us[i] << "\n";
    }
    std::cout << "[INFO] Saved " << full_us.size()
              << " frames to " << path << "\n";
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
    std::vector<double> full_us_vec;
    std::vector<double> transport_us_vec;
    full_us_vec.reserve(10000);
    transport_us_vec.reserve(10000);

    double   min_transport_us   = std::numeric_limits<double>::max();
    double   max_transport_us   = 0.0;
    uint64_t frame_count        = 0;
    int      previous_id        = -1;
//    int      skipped            = 0;


    const std::string csv_path = make_csv_path();
    RCLCPP_INFO(node->get_logger(),
        "FastDDS subscriber started — saving to %s", csv_path.c_str());

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

            // Need to change the image default message to also send the id of the image before doing this
            // Also need to add the logic to the publisher
//            if (msg->id - 1 > previous_id && previous_id != -1) {
//                skipped += static_cast<int>(msg->id - previous_id - 1);
//            }
            previous_id = static_cast<int>(msg->publish_timestamp);

            full_us_vec.push_back(full_us);
            transport_us_vec.push_back(transport_us);  
            
            total_full_us     += full_us;
            total_transport_us += transport_us;
            min_transport_us    = std::min(min_transport_us, transport_us);
            max_transport_us    = std::max(max_transport_us, transport_us);
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

            // ── Step 6: release chun  k back to iceoryx pool ────────────────────
            // Must be called — otherwise the pool exhausts and publisher stalls.
            iox_sub.release(userPayload);

        })
        .or_else([](auto &) {
            // no chunk available — yield to avoid starving other threads
            __asm__ volatile("pause" ::: "memory");  // CPU hint to reduce power/contention
        });
    }

    save_csv(csv_path, full_us_vec, transport_us_vec);

    // ── shutdown summary ──────────────────────────────────────────────────────
    if (frame_count > 0) {
        const double avg_full      = total_full_us      / static_cast<double>(frame_count);
        const double avg_transport = total_transport_us / static_cast<double>(frame_count);
        RCLCPP_INFO(node->get_logger(), "──────────────────────────────────────────");
        RCLCPP_INFO(node->get_logger(), "Transport time summary");
        RCLCPP_INFO(node->get_logger(), "  Frames received      : %lu",   frame_count);
        RCLCPP_INFO(node->get_logger(), "  Full latency   A→D   : %.1f us (%.3f ms)", avg_full,      avg_full      / 1000.0);
        RCLCPP_INFO(node->get_logger(), "  Transport only B→D   : %.1f us (%.3f ms)", avg_transport, avg_transport / 1000.0);
        RCLCPP_INFO(node->get_logger(), "  Transport min B→D    : %.1f us", min_transport_us);
        RCLCPP_INFO(node->get_logger(), "  Transport max B→D    : %.1f us", max_transport_us);
//        RCLCPP_INFO(node->get_logger(), "  Skipped frames       : %d",     skipped);
        RCLCPP_INFO(node->get_logger(), "  CSV saved to         : %s",     csv_path.c_str());
        RCLCPP_INFO(node->get_logger(), "──────────────────────────────────────────");
    } else {
        RCLCPP_WARN(node->get_logger(), "No frames received");
    }
    
    RCLCPP_INFO(node->get_logger(), "Shutting down native iceoryx subscriber");
    rclcpp::shutdown();
    return 0;
}