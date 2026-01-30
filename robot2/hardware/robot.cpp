#include "robot2/robot.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot2
{

    hardware_interface::CallbackReturn Robot4FarmersHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Initializing Robot4FarmersHardware");

        map_.clear();

        // Initialize internal maps for states and commands
        for (const auto &joint : info_.joints)
        {
            map_[joint.name] = Joint(joint.name);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Robot4FarmersHardware::on_configure(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring Robot4FarmersHardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Robot4FarmersHardware::on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating ...please wait... 1");

        for (auto &joint : map_)
        {

            if (joint.second.joint_name.find("rear") != std::string::npos)
            {
                joint.second.state.velocity = 0.5;
                joint.second.command.velocity = 0.5;
            }
            else if (joint.second.joint_name.find("front") != std::string::npos)
            {
                joint.second.state.position = 0.1;
                joint.second.command.position = 0.1;
            }
            else if (joint.second.joint_name.find("latitude") != std::string::npos)
            {
                joint.second.state.position = 0.0;
            }
            else if (joint.second.joint_name.find("longitude") != std::string::npos)
            {
                joint.second.state.position = 0.0;
            }
            else
            {
                joint.second.state.position = 0.0;
            }
        }

        RCLCPP_INFO(get_logger(), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Robot4FarmersHardware::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating Robot4FarmersHardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Robot4FarmersHardware::read(
        const rclcpp::Time &, const rclcpp::Duration &period)
    {
        // Here you could read sensors or encoders and update state_map_
        RCLCPP_DEBUG(get_logger(), "Reading state from Robot4FarmersHardware");

        for (auto &joint : map_)
        {
            if (joint.second.joint_name.find("front") != std::string::npos)
            {
                joint.second.state.position = joint.second.command.position;
            }
            else if (joint.second.joint_name.find("rear") != std::string::npos)
            {
                joint.second.state.velocity = joint.second.command.velocity;
                joint.second.state.position += joint.second.state.velocity * period.seconds();
            }
            else if (joint.second.joint_name.find("latitude") != std::string::npos)
            {
                joint.second.state.position = 0.0;
            }
            else if (joint.second.joint_name.find("longitude") != std::string::npos)
            {
                joint.second.state.position = 0.0;
            }
            else
            {

                joint.second.state.position = 0.0;
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Robot4FarmersHardware::write(
        const rclcpp::Time &, const rclcpp::Duration &period)
    {
        for (auto &joint : map_)
        {
            if (joint.second.joint_name.find("rear") != std::string::npos)
            {
                // Update rear wheel positions based on commanded velocity
                joint.second.state.velocity = joint.second.command.velocity;
                joint.second.state.position += joint.second.command.velocity * period.seconds();
            }
            else if (joint.second.joint_name.find("front") != std::string::npos)
            {
                // Update front wheel steering position
                joint.second.state.position = joint.second.command.position;
            }
        }
        RCLCPP_DEBUG(get_logger(), "Writing command to Robot4FarmersHardware");

        return hardware_interface::return_type::OK;
    }

    // Access functions used internally by the SystemInterface base
    double Robot4FarmersHardware::get_state(const std::string &name)
    {
        for (auto &joint : map_)
        {
            if (!joint.second.joint_name.compare(name))
                continue;
            if (name.find("rear") != std::string::npos)
            {
                return joint.second.state.velocity;
            }
            else
            {
                return joint.second.state.position;
            }
        }
    }

    void Robot4FarmersHardware::set_state(const std::string &name, double value)
    {
        for (auto &joint : map_)
        {
            if (!joint.second.joint_name.compare(name))
                continue;
            if (name.find("rear") != std::string::npos)
            {
                joint.second.state.velocity = value;
            }
            else if (joint.second.joint_name.find("front") != std::string::npos)
            {
                joint.second.state.position = value;
            }
            else if (joint.second.joint_name.find("latitude") != std::string::npos)
            {
                joint.second.state.position = value;
            }
            else if (joint.second.joint_name.find("longitude") != std::string::npos)
            {
                joint.second.state.position = value;
            }
            else
            {
                joint.second.state.position = value;
            }
        }
    }

    double Robot4FarmersHardware::get_command(const std::string &name)
    {
        for (auto &joint : map_)
        {
            if (!joint.second.joint_name.compare(name))
                continue;
            if (name.find("rear") != std::string::npos)
            {
                return joint.second.state.velocity;
            }
            else if (name.find("front") != std::string::npos)
            {
                return joint.second.state.position;
            }
        }
    }

    void Robot4FarmersHardware::set_command(const std::string &name, double value)
    {
        for (auto &joint : map_)
        {
            if (!joint.second.joint_name.compare(name))
                continue;
            if (name.find("rear") != std::string::npos)
            {
                joint.second.state.velocity = value;
            }
            else if (joint.second.joint_name.find("front") != std::string::npos)
            {
                joint.second.state.position = value;
            }
        }
    }

    rclcpp::Clock Robot4FarmersHardware::get_clock()
    {
        return clock_;
    }

    rclcpp::Logger Robot4FarmersHardware::get_logger()
    {
        return rclcpp::get_logger("Robot4FarmersHardware");
    }

    std::vector<hardware_interface::StateInterface> Robot4FarmersHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto &joint : map_)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

            if (joint.second.joint_name.find("rear") != std::string::npos)
            {
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
                        &joint.second.state.velocity));
            }
        }

        RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

        for (auto s : state_interfaces)
        {
            RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Robot4FarmersHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto &joint : map_)
        {
            if (joint.second.joint_name.find("front") != std::string::npos)
            {
                command_interfaces.emplace_back(
                    hardware_interface::CommandInterface(
                        joint.second.joint_name, hardware_interface::HW_IF_POSITION,
                        &joint.second.command.position));
            }
            else if (joint.first.find("rear") != std::string::npos)
            {
                command_interfaces.emplace_back(
                    hardware_interface::CommandInterface(
                        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
                        &joint.second.command.velocity));
            }
        }

        RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());

        for (auto i = 0u; i < command_interfaces.size(); i++)
        {
            RCLCPP_INFO(
                get_logger(), "Exported command interface '%s'.", command_interfaces[i].get_name().c_str());
        }
        return command_interfaces;
    }

} // namespace robot2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot2::Robot4FarmersHardware, hardware_interface::SystemInterface)
