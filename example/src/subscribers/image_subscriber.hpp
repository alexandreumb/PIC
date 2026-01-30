// Copyright (c) 2019 by Robert Bosch GmbH. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BURGER_SUBSCRIBER_HPP_
#define BURGER_SUBSCRIBER_HPP_

#include <inttypes.h>
#include <memory>
#include <string>
#include <csignal>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

template<class MsgT>
class BurgerSubscriber
{
public:
  BurgerSubscriber(std::shared_ptr<rclcpp::Node> node, std::string topic, bool show_gui = true)
  : node_(node),
    k_(0),
    average_round_time_(0),
    topic_(topic)
  {

    sub_ = node->create_subscription<MsgT>(
        topic,
        rclcpp::QoS(5).reliable(),
        [](std::shared_ptr<MsgT>){});
    
    image_buffer_.resize(12582912); // 2048 * 2048 * 3
  }

  virtual ~BurgerSubscriber()
  {
    
    auto logger = rclcpp::get_logger("image_transport_subscriber");
    RCLCPP_INFO(logger, "Received %" PRId64 " messages", k_);
    RCLCPP_INFO(
      logger, "Average round time %f milliseconds %s", static_cast<float>(average_round_time_/k_) / 1e6, topic_.c_str());
    if (!saved_)
    {
      RCLCPP_WARN(logger, "No image was saved, something went wrong!");
    }
  }

  void process_image(std::shared_ptr<MsgT> image)
  {
    saved_ = false;
    auto logger = rclcpp::get_logger("image_transport_subscriber");
    RCLCPP_INFO(logger, "Received message");  

    cv::Mat frame(image->height, image->width, CV_8UC3, image->data.data());

    auto msg_timestamp = image.get()->timestamp;
    auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    auto diff = now - msg_timestamp;  // convert Duration to int64_t nanoseconds
    average_round_time_ += diff;
    ++k_;
    
    cv::imwrite(std::string("/home/alexandre/image_subscriber") + topic_ + ".jpg", frame);
    saved_ = true;
  }

  typename rclcpp::Subscription<MsgT>::SharedPtr get_subscription() {return sub_;}

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub_;
  std::shared_ptr<rclcpp::Node> node_;
  int64_t k_;
  int64_t average_round_time_;
  std::string topic_;
  std::vector<uint8_t> image_buffer_;
  bool saved_ = false;
};

#endif  // BURGER_SUBSCRIBER_HPP_
