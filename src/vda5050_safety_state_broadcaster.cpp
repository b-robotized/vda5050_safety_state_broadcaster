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

#include "vda5050_safety_state_broadcaster/vda5050_safety_state_broadcaster.hpp"

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/helpers.hpp"

// namespace
// {  // utility

// // TODO(destogl): remove this when merged upstream
// // Changed services history QoS to keep all so we don't lose any client service calls
// static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
//   RMW_QOS_POLICY_HISTORY_KEEP_ALL,
//   1,  // message queue depth
//   RMW_QOS_POLICY_RELIABILITY_RELIABLE,
//   RMW_QOS_POLICY_DURABILITY_VOLATILE,
//   RMW_QOS_DEADLINE_DEFAULT,
//   RMW_QOS_LIFESPAN_DEFAULT,
//   RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
//   RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
//   false};

// using ControllerReferenceMsg =
// vda5050_safety_state_broadcaster::Vda5050SafetyStateBadcaster::ControllerReferenceMsg;

// // called from RT control loop
// void reset_controller_reference_msg(
//   std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
// {
//   msg->joint_names = joint_names;
//   msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
//   msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
//   msg->duration = std::numeric_limits<double>::quiet_NaN();
// }

// }  // namespace

namespace vda5050_safety_state_broadcaster
{

Vda5050SafetyStateBadcaster::Vda5050SafetyStateBadcaster()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn Vda5050SafetyStateBadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<vda5050_safety_state_broadcaster::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Vda5050SafetyStateBadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  try
  {
    vda5050_safety_state_publisher_ = get_node()->create_publisher<vda5050_msgs::msg::SafetyState>(
      "~/vda5050_safety_state", rclcpp::SystemDefaultsQoS());

    realtime_vda5050_safety_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<vda5050_msgs::msg::SafetyState>>(
        vda5050_safety_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Vda5050SafetyStateBadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
Vda5050SafetyStateBadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(
    params_.eStop_autoack_interfaces.size() + params_.eStop_manual_interfaces.size() +
    params_.eStop_remote_interfaces.size() + params_.fieldViolation_interfaces.size());
  for (auto const fieldViolation_interface : params_.fieldViolation_interfaces)
  {
    state_interfaces_config.names.push_back(fieldViolation_interface);
  }
  for (auto const eStop_manual_interface : params_.eStop_manual_interfaces)
  {
    state_interfaces_config.names.push_back(eStop_manual_interface);
  }
  for (auto const eStop_remote_interface : params_.eStop_remote_interfaces)
  {
    state_interfaces_config.names.push_back(eStop_remote_interface);
  }
  for (auto const eStop_autoack_interface : params_.eStop_autoack_interfaces)
  {
    state_interfaces_config.names.push_back(eStop_autoack_interface);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn Vda5050SafetyStateBadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces found to publish.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  auto & safety_state_msg = realtime_vda5050_safety_state_publisher_->msg_;

  safety_state_msg.e_stop = "none";
  safety_state_msg.field_violation = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Vda5050SafetyStateBadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Vda5050SafetyStateBadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  fieldViolation_value = false;
  estop_value = false;

  int i = 0;
  int j = static_cast<int>(params_.fieldViolation_interfaces.size());
  for (i; i < j; ++i)
  {
    fieldViolation_value |= this->safe_double_to_bool(state_interfaces_[i].get_value());
  }

  j += static_cast<int>(params_.eStop_manual_interfaces.size());
  for (i; i < j; ++i)
  {
    estop_value |= this->safe_double_to_bool(state_interfaces_[i].get_value());
  }

  if (estop_value)
  {
    estop_msg = "manual";
  }
  else
  {
    j += static_cast<int>(params_.eStop_remote_interfaces.size());
    for (i; i < j; ++i)
    {
      estop_value |= this->safe_double_to_bool(state_interfaces_[i].get_value());
    }
    if (estop_value)
    {
      estop_msg = "remote";
    }
    else
    {
      j += static_cast<int>(params_.eStop_autoack_interfaces.size());
      for (i; i < j; ++i)
      {
        estop_value |= this->safe_double_to_bool(state_interfaces_[i].get_value());
      }
      if (estop_value)
      {
        estop_msg = "autoack";
      }
      else
      {
        estop_msg = "none";
      }
    }
  }

  if (
    realtime_vda5050_safety_state_publisher_ && realtime_vda5050_safety_state_publisher_->trylock())
  {
    auto & safety_state_msg = realtime_vda5050_safety_state_publisher_->msg_;

    safety_state_msg.field_violation = fieldViolation_value;
    safety_state_msg.e_stop = estop_msg;
    realtime_vda5050_safety_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace vda5050_safety_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  vda5050_safety_state_broadcaster::Vda5050SafetyStateBadcaster,
  controller_interface::ControllerInterface)
