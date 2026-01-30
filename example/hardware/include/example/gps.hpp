#pragma once

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <serial/serial.h>
#include <unordered_map>
#include <string>

namespace example
{
    struct GPSData
    {
        double latitude{0.0};
        double longitude{0.0};
        double altitude{0.0};
    };
    struct Joint
    {
        explicit Joint(const std::string &name) : joint_name(name)
        {
            state = GPSData();
        }

        Joint() = default;

        std::string joint_name;
        GPSData state;
    };
    class ExampleHardware : public hardware_interface::SensorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ExampleHardware)

        // Lifecycle hooks
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override
        {
            RCLCPP_INFO(get_logger(), "Cleaning up Robot4FarmersHardware");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override
        {
            RCLCPP_INFO(get_logger(), "Shutting down Robot4FarmersHardware");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // Read/Write
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // Logger & Clock
        rclcpp::Clock get_clock();
        rclcpp::Logger get_logger();

        // State/Command access (map-based)
        double get_state(const std::string &name);
        void set_state(const std::string &name, double value);

    private:
        rclcpp::Clock clock_;
        serial::Serial serial_;

        // Maps for joint states and commands
        std::unordered_map<std::string, Joint> map_;
    };

} // namespace example

