// Copyright 2020 ROS2-Control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...
#include <errno.h> 
#include <algorithm> 
#include <chrono> 
#include <csignal> 
#include <memory> 
#include <string> 
#include <thread> 
#include <vector> 
#include <atomic> 
#include <iostream> 
#include <sys/timerfd.h>
#include <unistd.h>

#include "controller_manager/controller_manager.hpp" 
#include "rclcpp/rclcpp.hpp" 
#include "realtime_tools/realtime_helpers.hpp" 

using namespace std::chrono_literals; 
namespace { 
  int const kSchedPriority = 80; 
  std::atomic<bool> stop_loop(false); // Signal handler for Ctrl+C 
  
  void sigint_handler(int) 
  { 
    stop_loop = true; 
  } 
} // namespace 

int main(int argc, char **argv) { 
  rclcpp::init(argc, argv); // Register Ctrl+C handler 
  std::signal(SIGINT, sigint_handler); 
  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(); 
  std::string manager_node_name = "controller_manager"; 
  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name); 
  const bool use_sim_time = cm->get_parameter_or("use_sim_time", false); 
  
  const int cpu_affinity = cm->get_parameter_or<int>("cpu_affinity", -1); 
  if (cpu_affinity >= 0) { 
    const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity); 
    
    if (!affinity_result.first) { 
      RCLCPP_WARN( cm->get_logger(), "Unable to set the CPU affinity : '%s'", affinity_result.second.c_str()); 
    } 
  } 
  
  const bool has_realtime = realtime_tools::has_realtime_kernel(); 
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", has_realtime); 
  
  if (lock_memory) { 
    const auto lock_result = realtime_tools::lock_memory(); 
    
    if (!lock_result.first) { 
      RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory: '%s'", lock_result.second.c_str()); 
    } 
  } 
  
  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate()); 
  
  const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority); 
  RCLCPP_INFO(cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(), thread_priority); 
  const double target_ms = 10.0; // 50 Hz 
  
std::thread cm_thread([cm, thread_priority, use_sim_time, target_ms, cpu_affinity]() 
{     
    size_t iteration = 0;  // Add this back for tracking
    
    // Set CPU affinity FIRST (before RT scheduling)
    if (cpu_affinity >= 0) {
        const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity);
        
        if (!affinity_result.first) {
            RCLCPP_WARN(cm->get_logger(), "Unable to set the CPU affinity: '%s'", affinity_result.second.c_str());
        } else {
            RCLCPP_INFO(cm->get_logger(), "CPU affinity set to core %d", cpu_affinity);
        }
    }
    
    // THEN set RT scheduling
    if (realtime_tools::has_realtime_kernel()) 
    { 
        if (!realtime_tools::configure_sched_fifo(thread_priority)) { 
            RCLCPP_WARN(cm->get_logger(), "Could not enable FIFO RT scheduling policy: with error number <%i>(%s)...", errno, strerror(errno)); 
        } 
        else 
        { 
            RCLCPP_INFO(cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.", thread_priority); 
        } 
    } 
    else 
    { 
        RCLCPP_WARN(cm->get_logger(), "No real-time kernel detected on this system..."); 
    }
    
    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (tfd < 0) {
        RCLCPP_ERROR(cm->get_logger(), "timerfd_create failed");
        return;
    }

    auto const period_ns = 1'000'000'000LL / cm->get_update_rate();
    auto previous_time = std::chrono::steady_clock::now(); 
    auto time_now = std::chrono::duration_cast<std::chrono::nanoseconds>(previous_time.time_since_epoch()).count();
    auto first_abs_ns = time_now + period_ns;

    itimerspec ts{};
    ts.it_value.tv_sec  = first_abs_ns / 1'000'000'000LL;
    ts.it_value.tv_nsec = first_abs_ns % 1'000'000'000LL;
    ts.it_interval.tv_sec  = period_ns / 1'000'000'000LL;
    ts.it_interval.tv_nsec = period_ns % 1'000'000'000LL;
    
    uint64_t expirations;

    if (timerfd_settime(tfd, TFD_TIMER_ABSTIME, &ts, nullptr) < 0) 
    {
        RCLCPP_ERROR(cm->get_logger(), "timerfd_settime failed: %s", strerror(errno));
        close(tfd);
        return;
    }

    while (rclcpp::ok() && !stop_loop) 
    {
        ssize_t n = read(tfd, &expirations, sizeof(expirations));
        if (n < 0) break;

        auto current_time = std::chrono::steady_clock::now(); 
        auto measured_period = std::chrono::duration<double, std::milli>(current_time - previous_time).count(); 
        previous_time = current_time;
        
        iteration++;  // Increment iteration counter
        
        // Check for missed deadlines
        if (expirations > 1) {
            RCLCPP_WARN(cm->get_logger(), "Missed %llu deadline(s) at iteration %zu", expirations - 1, iteration);
        }
        
        rclcpp::Time ros_now = cm->now();
        rclcpp::Duration dt(std::chrono::nanoseconds(static_cast<int64_t>(measured_period * 1e6)));

        cm->read(ros_now, dt); 
        cm->update(ros_now, dt); 
        cm->write(ros_now, dt); 
    }
    
    close(tfd);
    
    RCLCPP_INFO(cm->get_logger(), "Control loop completed %zu iterations", iteration);
});

  executor->add_node(cm); 
  executor->spin(); 
  cm_thread.join(); 
  rclcpp::shutdown(); 
  return 0; 
}