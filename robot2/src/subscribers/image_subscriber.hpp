#ifndef BURGER_SUBSCRIBER_HPP_
#define BURGER_SUBSCRIBER_HPP_

#include <inttypes.h>
#include <memory>
#include <string>
#include <csignal>
#include <vector>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

template<class MsgT>
class BurgerSubscriber
{
public:
  struct FreqSample
  {
    int32_t frequency;
    int64_t diff_ns;
  };

  BurgerSubscriber(std::shared_ptr<rclcpp::Node> node, std::string topic, bool show_gui = true)
  : node_(node),
    k_(0),
    average_round_time_(0),
    topic_(topic)
  {
    sub_ = node->create_subscription<MsgT>(
        topic,
        rclcpp::QoS(5).best_effort(),
        [this](std::shared_ptr<MsgT> msg){ this->process_image(msg); });

    image_buffer_.resize(12582912); // 2048 * 2048 * 3
  }

  virtual ~BurgerSubscriber()
  {
    auto logger = rclcpp::get_logger("image_transport_subscriber");
    RCLCPP_INFO(logger, "Received %" PRId64 " messages", k_);
    if (k_ > 0)
    {
      RCLCPP_INFO(
        logger, "Average round time %f ms %s",
        static_cast<float>(average_round_time_/k_) / 1e6,
        topic_.c_str());
    }

    // Save frequency log to CSV
    if (!freq_log_.empty())
    {
      std::ofstream csv("/home/alexandre/frequency_log_" + topic_ + ".csv");
      csv << "frequency,diff_ns,diff_ms\n";
      for (const auto& s : freq_log_)
      {
        csv << s.frequency << "," << s.diff_ns << "," << (s.diff_ns / 1e6) << "\n";
      }
      csv.close();
      RCLCPP_INFO(logger, "Saved %zu frequency samples to CSV", freq_log_.size());
    }
    else
    {
      RCLCPP_WARN(logger, "No frequency data to save for topic %s", topic_.c_str());
    }

    if (!saved_)
    {
      RCLCPP_WARN(logger, "No image was saved, something went wrong!");
    }
  }

  void process_image(std::shared_ptr<MsgT> image)
  {
    saved_ = false;
    auto logger = rclcpp::get_logger("image_transport_subscriber");
    
    cv::Mat frame(image->height, image->width, CV_8UC3, image->data.data());

    // --- Timestamp calculation ---
    auto msg_timestamp = image->timestamp;
    auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    
    auto diff = now - msg_timestamp;  // nanoseconds
    average_round_time_ += diff;
    ++k_;

    // --- Save frequency info ---
    int32_t freq = 0;
    if constexpr (std::is_member_object_pointer<decltype(&MsgT::frequency)>::value)
    {
      freq = image->frequency;
    }
    freq_log_.push_back({freq, diff});

    // --- Save image ---
    saved_ = true;

  }

  typename rclcpp::Subscription<MsgT>::SharedPtr get_subscription() { return sub_; }

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub_;
  std::shared_ptr<rclcpp::Node> node_;
  int64_t k_;
  int64_t average_round_time_;
  std::string topic_;
  std::vector<uint8_t> image_buffer_;
  bool saved_ = false;

  std::vector<FreqSample> freq_log_;
};

#endif  // BURGER_SUBSCRIBER_HPP_
