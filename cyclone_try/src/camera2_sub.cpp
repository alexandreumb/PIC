#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "fixed_size_msgs/msg/image8_mb.hpp"

class SHMSubscriber : public rclcpp::Node
{
public:
  SHMSubscriber() : Node("shm_subscriber")
  {
    sub_ = create_subscription<fixed_size_msgs::msg::Image8Mb>(
      "shm_test",
      rclcpp::QoS(10),
      std::bind(&SHMSubscriber::callback, this, std::placeholders::_1)
    );
  }

private:
  void callback(const std::shared_ptr<fixed_size_msgs::msg::Image8Mb> msg) {
    cv::Mat frame(msg->height, msg->width, CV_8UC3, msg->data.data());

    auto msg_timestamp = msg->timestamp;
    auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    auto diff = now - msg_timestamp;  // nanoseconds
    average_round_time_ += diff;
    ++k_;

    RCLCPP_INFO(this->get_logger(), "Processed message %ld, diff %f ms", k_, (average_round_time_/1e6)/k_);
  }

  rclcpp::Subscription<fixed_size_msgs::msg::Image8Mb>::SharedPtr sub_;
  int64_t k_ = 0;
  int64_t average_round_time_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SHMSubscriber>());
  rclcpp::shutdown();
  return 0;
}
