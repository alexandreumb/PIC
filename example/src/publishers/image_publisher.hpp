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

#ifndef BURGER_PUBLISHER_HPP_
#define BURGER_PUBLISHER_HPP_

#include <inttypes.h>
#include <memory>
#include <string>
#include <utility>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

template<class MsgT>
class BurgerPublisher
{
public:
  BurgerPublisher(std::shared_ptr<rclcpp::Node> node, std::string topic, bool robot)
  : pub_(node->create_publisher<MsgT>(topic, 1)),
    logger_(node->get_logger()),
    count_(0),
    robot_(robot)
  {
    if (!robot)
    {
      img_ = cv::imread("/home/alexandre/large_image.jpg", cv::IMREAD_COLOR);
      if (img_.empty()) {
        RCLCPP_ERROR(logger_, "Could not open or find the image!");
        return;
      }
      frame_size_ = static_cast<size_t>(img_.step[0] * img_.rows);
    } else
    {
      cap_.open(0);
      if (!cap_.isOpened())
      {
        RCLCPP_ERROR(logger_, "Failed to open camera!");
      }
      
      cap_.set(cv::CAP_PROP_FPS, 30);
    }
  }

  void publish_image()
  {

    auto image_msg = pub_->borrow_loaned_message();
    auto msg_size = image_msg.get().data.size();

    if (!robot_) 
    {
      if (frame_size_ > msg_size) {
        RCLCPP_ERROR(
          logger_, "incompatible image sizes. frame %zu, msg %zu", frame_size_, msg_size);
        return;
      }
    } else {
      cap_ >> img_;
      if (img_.empty()) return;

      frame_size_ = static_cast<size_t>(img_.step[0] * img_.rows);
      if (frame_size_ > msg_size) {
        RCLCPP_ERROR(
          logger_, "incompatible image sizes. frame %zu, msg %zu", frame_size_, msg_size);
        return;
      }
    }

    RCLCPP_INFO(logger_, "frame_size_ = %zu, msg.data.size() = %zu", frame_size_, msg_size);

    auto & msg = image_msg.get();
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(img_.cols * img_.elemSize());
    msg.width = img_.cols;
    msg.height = img_.rows;
    memcpy(msg.data.data(), img_.data, frame_size_);

    image_msg.get().timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    pub_->publish(std::move(image_msg));
    ++count_;
  }

  typename rclcpp::Publisher<MsgT>::SharedPtr get_publisher() { return pub_; }


private:
  std::shared_ptr<rclcpp::Publisher<MsgT>> pub_;
  rclcpp::Logger logger_;
  cv::Mat img_;
  cv::VideoCapture cap_;
  size_t frame_size_;
  size_t count_;
  bool robot_;
};

#endif  // BURGER_PUBLISHER_HPP_
