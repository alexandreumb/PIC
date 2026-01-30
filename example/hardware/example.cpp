#include "example/gps.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace example
{

    hardware_interface::CallbackReturn ExampleHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SensorInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Initializing ExampleHardware");

        map_.clear();

        // Initialize internal maps for states
        for (const auto &joint : info_.joints)
        {
            RCLCPP_INFO(get_logger(), "Adding joint '%s' to the hardware interface.", joint.name.c_str());
            map_[joint.name] = Joint(joint.name);
        }
        for (const auto &sensor : info_.sensors)
        {
            RCLCPP_INFO(get_logger(), "Adding sensor '%s' to the hardware interface.", sensor.name.c_str());
            map_[sensor.name] = Joint(sensor.name);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ExampleHardware::on_configure(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring ExampleHardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ExampleHardware::on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating ...please wait... 1");

        for (auto &joint : map_)
        {

            if (joint.second.joint_name.find("latitude") != std::string::npos)
            {
                joint.second.state.latitude = 0.0;
            }
            else if (joint.second.joint_name.find("longitude") != std::string::npos)
            {
                joint.second.state.longitude = 0.0;
            }
            else if (joint.second.joint_name.find("altitude") != std::string::npos)
            {
                joint.second.state.altitude = 0.0;
            }
        }

        RCLCPP_INFO(get_logger(), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ExampleHardware::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating ExampleHardware");
        // optionally reset internal state here
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ExampleHardware::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Here you could read sensors and update state_map_
        RCLCPP_DEBUG(get_logger(), "Reading state from ExampleHardware");

        for (auto &joint : map_)
        {
            if (joint.second.joint_name.find("gps_sensor") != std::string::npos)
            {
                joint.second.state.latitude += 0.1;
                joint.second.state.longitude += 0.1;
                joint.second.state.altitude += 0.1;
            }
        }

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> ExampleHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto &joint : map_)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "latitude", &joint.second.state.latitude));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "longitude", &joint.second.state.longitude));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "altitude", &joint.second.state.altitude));
        }

        RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

        for (auto s : state_interfaces)
        {
            RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
        }

        return state_interfaces;
    }

    rclcpp::Clock ExampleHardware::get_clock()
    {
        return rclcpp::Clock(RCL_ROS_TIME);
    }

    rclcpp::Logger ExampleHardware::get_logger()
    {
        return rclcpp::get_logger("ExampleHardware");
    }

    double ExampleHardware::get_state(const std::string &name)
    {
        for (const auto &joint : map_)
        {
            if (joint.second.joint_name == name)
            {
                return joint.second.state.latitude; // Example: return latitude
            }
        }
        return 0.0;
    }

    void ExampleHardware::set_state(const std::string &name, double value)
    {
        for (auto &joint : map_)
        {
            if (joint.second.joint_name == name)
            {
                joint.second.state.latitude = value; // Example: set latitude
            }
        }
    }

} // namespace gps

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example::ExampleHardware, hardware_interface::SensorInterface)
