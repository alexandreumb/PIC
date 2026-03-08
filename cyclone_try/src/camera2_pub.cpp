#include <rclcpp/rclcpp.hpp>
#include "fixed_size_msgs/msg/image8_mb.hpp"
#include <opencv2/opencv.hpp>

class SHMPublisher : public rclcpp::Node
{
public:
  SHMPublisher() : Node("shm_publisher")
  {
    pub_ = create_publisher<fixed_size_msgs::msg::Image8Mb>(
      "shm_test", rclcpp::QoS(10));

    color_img_ = cv::imread("/home/alexandre/large_image.jpg", cv::IMREAD_COLOR);
    color_size_ = static_cast<size_t>(color_img_.step[0] * color_img_.rows);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SHMPublisher::publish_msg, this));
  }

private:
  void publish_msg()
  {
    msg_.is_bigendian = false;
    msg_.step = static_cast<uint32_t>(color_img_.cols * color_img_.elemSize());
    msg_.width = color_img_.cols;
    msg_.height = color_img_.rows;
    msg_.frequency = 0;
    std::memcpy(msg_.data.data(), color_img_.data, color_size_);
    msg_.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch())
                    .count();
    pub_->publish(msg_);
  }

    cv::Mat color_img_;
    int32_t color_size_ = 0;
    rclcpp::Publisher<fixed_size_msgs::msg::Image8Mb>::SharedPtr pub_;
    fixed_size_msgs::msg::Image8Mb msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SHMPublisher>());
  rclcpp::shutdown();
  return 0;
}
