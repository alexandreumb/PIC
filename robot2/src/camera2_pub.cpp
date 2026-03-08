#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include "publishers/image_publisher.hpp"
#include "publishers/image_publisher_gpu.hpp"
#include "iceoryx_posh/popo/publisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("camera2_node");
    auto pub = std::make_shared<BurgerPublisher<fixed_size_msgs::msg::Image8Mb>>(node, "camera2", true);
    int32_t frequency = 30;
    std::unique_ptr<rclcpp::Rate> rate = std::make_unique<rclcpp::Rate>(frequency);

    while (rclcpp::ok())
    {
        pub->publish_image(frequency);
        rate->sleep();
    }

    rclcpp::shutdown();

    return 0;
}

/*
#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include "publishers/image_publisher.hpp"
#include "publishers/image_publisher_gpu.hpp"
#include "publishers/image_pub.hpp"
#include <chrono>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    bool robot = false;
    auto val = 0;

    rclcpp::Rate rate(robot ? 1000 : 30)

    if (val == 0)
    {
        iox::runtime::PoshRuntime::initRuntime("camera1_pub");
        image_pub<fixed_size_msgs::msg::Image8Mb> pub({"camera1", "Image", "8Mb"}, robot);
        while (rclcpp::ok())
        {
            pub.send_image();
            if (!robot)
            {
                rate.sleep();
            }
        }
    }
    else
    {
        iox::runtime::PoshRuntime::initRuntime("camera1_pub");
        image_publisher_gpu<fixed_size_msgs::msg::Image8Mb> pub(
            iox::capro::ServiceDescription{"camera1", "Image", "8Mb"});
        while (rclcpp::ok())
        {
            pub.send_image();
        }
    }

    rclcpp::shutdown();

    return 0;
}
*/