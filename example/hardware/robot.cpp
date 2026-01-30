#include "example/robot.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace example
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

    joints_.clear();
    
    // CRITICAL: Reserve exact size and fill with emplace_back
    joints_.reserve(info_.joints.size());

    for (const auto &joint : info_.joints)
    {
        joints_.emplace_back(joint.name);  // Use emplace_back, not push_back
        joints_.back().state.position = 0.0;
        joints_.back().state.velocity = 0.0;
        joints_.back().command.position = 0.0;
        joints_.back().command.velocity = 0.0;
    }
    
    RCLCPP_INFO(get_logger(), "Initialized %zu joints", joints_.size());
    
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
    RCLCPP_INFO(get_logger(), "Activating Robot4FarmersHardware");

    for (auto &joint : joints_)
    {
        joint.state.position = joint.command.position;
        joint.state.velocity = joint.command.velocity;
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
    for (auto &joint : joints_)
    {
        if (joint.joint_name.find("front") != std::string::npos)
        {
            joint.state.position = joint.command.position;
        }
        else if (joint.joint_name.find("rear") != std::string::npos)
        {
            joint.state.position += joint.state.velocity * period.seconds();
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Robot4FarmersHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name.find("rear") != std::string::npos)
        {
            joint.state.velocity = joint.command.velocity;
        }
        else if (joint.joint_name.find("front") != std::string::npos)
        {
            joint.state.position = joint.command.position;
        }
    }
    return hardware_interface::return_type::OK;
}

// Helper functions
double Robot4FarmersHardware::get_state(const std::string &name)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name == name)
        {
            if (name.find("rear") != std::string::npos)
            {
                return joint.state.velocity;
            }
            else
            {
                return joint.state.position;
            }
        }
    }
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in state", name.c_str());
    return 0.0;
}

void Robot4FarmersHardware::set_state(const std::string &name, double value)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name == name)
        {
            if (name.find("rear") != std::string::npos)
            {
                joint.state.velocity = value;
            }
            else if (name.find("front") != std::string::npos)
            {
                joint.state.position = value;
            }
            return;
        }
    }
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in state", name.c_str());
}

double Robot4FarmersHardware::get_command(const std::string &name)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name == name)
        {
            if (name.find("rear") != std::string::npos)
            {
                return joint.command.velocity;
            }
            else if (name.find("front") != std::string::npos)
            {
                return joint.command.position;
            }
        }
    }
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in command", name.c_str());
    return 0.0;
}

void Robot4FarmersHardware::set_command(const std::string &name, double value)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name == name)
        {
            if (name.find("rear") != std::string::npos)
            {
                joint.command.velocity = value;
            }
            else if (name.find("front") != std::string::npos)
            {
                joint.command.position = value;
            }
            return;
        }
    }
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in command", name.c_str());
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

    // Use index-based loop instead of range-based
    for (size_t i = 0; i < joints_.size(); i++)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joints_[i].joint_name, 
                hardware_interface::HW_IF_POSITION, 
                &joints_[i].state.position));

        if (joints_[i].joint_name.find("rear") != std::string::npos)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joints_[i].joint_name, 
                    hardware_interface::HW_IF_VELOCITY,
                    &joints_[i].state.velocity));
        }
    }

    RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Robot4FarmersHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Use index-based loop instead of range-based
    for (size_t i = 0; i < joints_.size(); i++)
    {
        if (joints_[i].joint_name.find("front") != std::string::npos)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joints_[i].joint_name, 
                    hardware_interface::HW_IF_POSITION,
                    &joints_[i].command.position));
        }
        else if (joints_[i].joint_name.find("rear") != std::string::npos)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joints_[i].joint_name, 
                    hardware_interface::HW_IF_VELOCITY,
                    &joints_[i].command.velocity));
        }
    }

    RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());
    return command_interfaces;
}

} // namespace example

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example::Robot4FarmersHardware, hardware_interface::SystemInterface)