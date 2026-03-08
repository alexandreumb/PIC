#include "fixed_size_msgs/msg/image8_mb.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"

#include "subscribers/image_subscriber.hpp"
#include "subscribers/image_subscriber_gpu.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("camera2_sub_node");
    auto sub = std::make_shared<BurgerSubscriber<fixed_size_msgs::msg::Image8Mb>>(node, "camera2");

    rclcpp::Rate rate(500);

    rclcpp::MessageInfo info;
    auto msg = std::make_shared<fixed_size_msgs::msg::Image8Mb>();
    auto sub_ = sub->get_subscription();

    // --- Wait for publisher before starting ---
    while (rclcpp::ok() && sub_->get_publisher_count() == 0)
    {
        RCLCPP_INFO(node->get_logger(), "Waiting for publisher...");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // --- Main receive loop ---
    while (rclcpp::ok())
    {
        try
        {
            if (sub_->take(*msg, info))
            {
                sub->process_image(msg);
            }
        }
        catch (const rclcpp::exceptions::RCLError &e)
        {
            std::string err = e.what();
            // Ignore this specific Iceoryx transient error
            if (err.find("No chunk in iceoryx_receiver") != std::string::npos)
            {
                // Just continue — this happens when there’s no message available
            }
            else
            {
                // For any other unexpected RCL error, log but don’t crash
                RCLCPP_ERROR(node->get_logger(), "RCL error: %s", e.what());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Unexpected exception: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(node->get_logger(), "Unknown exception caught");
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
