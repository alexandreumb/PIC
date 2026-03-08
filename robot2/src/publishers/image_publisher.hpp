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
      cap_.open(2);
      if (!cap_.isOpened())
      {
        RCLCPP_ERROR(logger_, "Failed to open camera!");
      }
      
      cap_.set(cv::CAP_PROP_FPS, 30);
    }
  }

  void publish_image(int32_t freq)
    {
      auto image_msg = pub_->borrow_loaned_message();
      auto & msg = image_msg.get();
      auto msg_size = msg.data.size();

      if (!robot_) 
      {
        if (frame_size_ > msg_size) {
          RCLCPP_ERROR(
            logger_, "incompatible image sizes. frame %zu, msg %zu", frame_size_, msg_size);
          return;
        }

        msg.is_bigendian = false;
        msg.step = static_cast<uint32_t>(img_.cols * img_.elemSize());
        msg.width = img_.cols;
        msg.height = img_.rows;
        msg.frequency = freq;
        memcpy(msg.data.data(), img_.data, frame_size_);

      } else {

        // Get frame dimensions from first capture to validate size
        if (width_ == 0) {
          cv::Mat tmp;
          cap_ >> tmp;
          if (tmp.empty()) return;
          width_ = tmp.cols;
          height_ = tmp.rows;
          frame_size_ = static_cast<size_t>(tmp.step[0] * tmp.rows);
          if (frame_size_ > msg_size) {
            RCLCPP_ERROR(
              logger_, "incompatible image sizes. frame %zu, msg %zu", frame_size_, msg_size);
            return;
          }
        }

        // Wrap shared memory buffer as cv::Mat — OpenCV writes directly into shared memory
        cv::Mat frame_in_shm(height_, width_, CV_8UC3, msg.data.data());
        cap_ >> frame_in_shm;  // zero-copy: camera writes directly into shared memory
        if (frame_in_shm.empty()) return;

        msg.is_bigendian = false;
        msg.step = static_cast<uint32_t>(width_ * frame_in_shm.elemSize());
        msg.width = width_;
        msg.height = height_;
        msg.frequency = freq;
        frame_size_ = static_cast<size_t>(msg.step * height_);
      }

      RCLCPP_INFO(logger_, "frame_size_ = %zu, msg.data.size() = %zu", frame_size_, msg_size);

      msg.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

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
  int width_ = 0;   // add this
  int height_ = 0;  // add this
};

#endif  // BURGER_PUBLISHER_HPP_
