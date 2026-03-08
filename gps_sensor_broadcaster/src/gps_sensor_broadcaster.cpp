// Copyright 2024
// Apache 2.0 License

#include "gps_sensor_broadcaster/gps_sensor_broadcaster.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace gps_sensor_broadcaster
{

  GPSSensorBroadcaster::GPSSensorBroadcaster() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn GPSSensorBroadcaster::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Init failed: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  GPSSensorBroadcaster::command_interface_configuration() const
  {
    // GPS has NO command interfaces
    return {
        controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration
  GPSSensorBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Expect exactly these state interfaces
    for (const auto &sensor : params_.sensors)
    {
      cfg.names.push_back(sensor + "/latitude");
      cfg.names.push_back(sensor + "/longitude");
      cfg.names.push_back(sensor + "/altitude");
    }

    return cfg;
  }

  controller_interface::CallbackReturn
  GPSSensorBroadcaster::on_configure(const rclcpp_lifecycle::State &)
  {
    gps_sensor_publisher_ =
        get_node()->create_publisher<sensor_msgs::msg::NavSatFix>(
            "~/fix", rclcpp::SensorDataQoS());

    realtime_gps_sensor_publisher_ =
        std::make_shared<
            realtime_tools::RealtimePublisher<sensor_msgs::msg::NavSatFix>>(
            gps_sensor_publisher_);

    RCLCPP_INFO(get_node()->get_logger(), "GPS Sensor Broadcaster configured");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  GPSSensorBroadcaster::on_activate(const rclcpp_lifecycle::State &)
  {
    if (state_interfaces_.size() < 3)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Expected at least 3 state interfaces (lat, lon, alt)");
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "GPS Sensor Broadcaster activated");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  GPSSensorBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type
  GPSSensorBroadcaster::update(
      const rclcpp::Time &time,
      const rclcpp::Duration &)
  {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    for (const auto &si : state_interfaces_)
    {
      if (si.get_interface_name() == "latitude")
      {
        latitude = si.get_value();
      }
      else if (si.get_interface_name() == "longitude")
      {
        longitude = si.get_value();
      }
      else if (si.get_interface_name() == "altitude")
      {
        altitude = si.get_value();
      }
    }

    if (realtime_gps_sensor_publisher_ && realtime_gps_sensor_publisher_->trylock())
    {
      auto &msg = realtime_gps_sensor_publisher_->msg_;
      msg.header.stamp = time;
      msg.header.frame_id = params_.frame_id;

      msg.latitude = latitude;
      msg.longitude = longitude;
      msg.altitude = altitude;

      // Optional but recommended
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

      realtime_gps_sensor_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

} // namespace gps_sensor_broadcaster

PLUGINLIB_EXPORT_CLASS(gps_sensor_broadcaster::GPSSensorBroadcaster, controller_interface::ControllerInterface)
