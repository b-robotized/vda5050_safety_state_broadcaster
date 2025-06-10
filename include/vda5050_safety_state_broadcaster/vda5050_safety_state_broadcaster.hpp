// Copyright (c) 2025, b-robotized
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_
#define VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "vda5050_msgs/msg/safety_state.hpp"
#include "vda5050_safety_state_broadcaster_parameters.hpp"

namespace vda5050_safety_state_broadcaster
{

/**
 * \brief vda5050_safety_state_broadcaster for all or some state in a ros2_control system.
 *
 * Vda5050SafetyStateBadcaster publishes state interfaces from ros2_control as ROS messages.
 * The following state interfaces are published:
 *    <state_joint>/xxxxx
 *
 * \param xxxxxx
 *
 * Publishes to:
 *
 * - \b xxxx (xxxx::msg::xxx): xxxx
 *
 */
class Vda5050SafetyStateBadcaster : public controller_interface::ControllerInterface
{
public:
  Vda5050SafetyStateBadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool safe_double_to_bool(double value)
  {
    if (std::isnan(value))
    {
      return false;
    }
    return value != 0.0;
  }

protected:
  std::shared_ptr<vda5050_safety_state_broadcaster::ParamListener> param_listener_;
  vda5050_safety_state_broadcaster::Params params_;

  std::shared_ptr<rclcpp::Publisher<vda5050_msgs::msg::SafetyState>>
    vda5050_safety_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<vda5050_msgs::msg::SafetyState>>
    realtime_vda5050_safety_state_publisher_;

  bool fieldViolation_value = false;
  bool estop_value = false;
  std::string estop_msg = "none";

private:
};

}  // namespace vda5050_safety_state_broadcaster

#endif  // VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_
