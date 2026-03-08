// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef FOUR_WHEELS_STEERING_CONTROLLER__FOUR_WHEELS_STEERING_CONTROLLER_HPP_
#define FOUR_WHEELS_STEERING_CONTROLLER__FOUR_WHEELS_STEERING_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "four_wheels_steering_controller/visibility_control.h"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/steering_controller_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "four_wheels_steering_controller/four_wheels_steering_controller_parameters.hpp"
#include "steering_controllers_library/steering_odometry.hpp"

namespace four_wheels_steering_controller
{

// name constants for state interfaces
static constexpr size_t STATE_TRACTION_RIGHT_WHEEL = 0;
static constexpr size_t STATE_TRACTION_LEFT_WHEEL = 1;
static constexpr size_t STATE_STEER_RIGHT_WHEEL = 2;
static constexpr size_t STATE_STEER_LEFT_WHEEL = 3;

// name constants for command interfaces
static constexpr size_t CMD_TRACTION_RIGHT_WHEEL = 0;
static constexpr size_t CMD_TRACTION_LEFT_WHEEL = 1;
static constexpr size_t CMD_STEER_RIGHT_WHEEL = 2;
static constexpr size_t CMD_STEER_LEFT_WHEEL = 3;

static constexpr size_t NR_STATE_ITFS = 4;
static constexpr size_t NR_CMD_ITFS = 4;
static constexpr size_t NR_REF_ITFS = 2;

class FourWheelsSteeringController : public controller_interface::ChainableControllerInterface
{
public:
  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC FourWheelsSteeringController();

  virtual FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC void
  initialize_implementation_parameter_listener() = 0;

  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_init() override;

  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  virtual FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn
  configure_odometry() = 0;

  virtual FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC bool update_odometry(
    const rclcpp::Duration & period) = 0;

  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::return_type
  update_reference_from_subscribers() override;

  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::return_type
  update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerTwistReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
  using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;
  using SteeringControllerStateMsg = control_msgs::msg::SteeringControllerStatus;
  using AckermannControllerState [[deprecated]] =
    SteeringControllerStateMsg;  // unused, but kept for backwards compatibility

protected:
  controller_interface::CallbackReturn set_interface_numbers(
    size_t nr_state_itfs, size_t nr_cmd_itfs, size_t nr_ref_itfs);

  std::shared_ptr<four_wheels_steering_controller::ParamListener> param_listener_;
  four_wheels_steering_controller::Params params_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerTwistReferenceMsg>::SharedPtr ref_subscriber_twist_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_subscriber_unstamped_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerTwistReferenceMsg>> input_ref_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);  // 0ms

  using ControllerStatePublisherOdom = realtime_tools::RealtimePublisher<ControllerStateMsgOdom>;
  using ControllerStatePublisherTf = realtime_tools::RealtimePublisher<ControllerStateMsgTf>;

  rclcpp::Publisher<ControllerStateMsgOdom>::SharedPtr odom_s_publisher_;
  rclcpp::Publisher<ControllerStateMsgTf>::SharedPtr tf_odom_s_publisher_;

  std::unique_ptr<ControllerStatePublisherOdom> rt_odom_state_publisher_;
  std::unique_ptr<ControllerStatePublisherTf> rt_tf_odom_state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  /// Odometry:
  steering_odometry::SteeringOdometry odometry_;

  SteeringControllerStateMsg published_state_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<SteeringControllerStateMsg>;
  rclcpp::Publisher<SteeringControllerStateMsg>::SharedPtr controller_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  // name constants for state interfaces
  size_t nr_state_itfs_;
  // name constants for command interfaces
  size_t nr_cmd_itfs_;
  // name constants for reference interfaces
  size_t nr_ref_itfs_;

  // last velocity commands for open loop odometry
  double last_linear_velocity_ = 0.0;
  double last_angular_velocity_ = 0.0;

  std::vector<std::string> rear_wheels_state_names_;
  std::vector<std::string> front_wheels_state_names_;

private:
  // callback for topic interface
  FOUR_WHEELS_STEERING_CONTROLLER__VISIBILITY_LOCAL void reference_callback(
    const std::shared_ptr<ControllerTwistReferenceMsg> msg);
  void reference_callback_unstamped(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
};

}  // namespace four_wheels_steering_controller

#endif  // FOUR_WHEELS_STEERING_CONTROLLER__FOUR_WHEELS_STEERING_CONTROLLER_HPP_
