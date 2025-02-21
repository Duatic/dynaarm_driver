/*
 * Copyright 2024 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <dynaarm_controllers/dynaarm_status_broadcaster.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace dynaarm_controllers
{

StatusBroadcaster::StatusBroadcaster()
{
}

controller_interface::InterfaceConfiguration StatusBroadcaster::command_interface_configuration() const
{
  // Claim the necessary command interfaces
  // In our case that are none as this is just a status controller
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::InterfaceConfiguration StatusBroadcaster::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    config.names.emplace_back(joint + "/motor_temperature_system");
    config.names.emplace_back(joint + "/motor_temperature_coil_A");
    config.names.emplace_back(joint + "/motor_temperature_coil_B");
    config.names.emplace_back(joint + "/motor_temperature_coil_C");
    config.names.emplace_back(joint + "/motor_bus_voltage");

    config.names.emplace_back(joint + "/position_commanded");
    config.names.emplace_back(joint + "/velocity_commanded");
    config.names.emplace_back(joint + "/effort_commanded");
  }
  return config;
}

controller_interface::CallbackReturn StatusBroadcaster::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<dynaarm_status_broadcaster::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
StatusBroadcaster::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // check if joints are empty
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // setup publishers and services
  arm_state_pub_ = get_node()->create_publisher<ArmState>("~/state", 10);  // TODO(firesurfer) what is the right qos ?
  arm_state_pub_rt_ = std::make_unique<ArmStatePublisher>(arm_state_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
StatusBroadcaster::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // first step is always clearing the interface lists
  // otherwise one gets interesting errors during reactivation
  joint_position_interfaces_.clear();
  joint_velocity_interfaces_.clear();
  joint_effort_interfaces_.clear();
  joint_position_commanded_interfaces_.clear();
  joint_velocity_commanded_interfaces_.clear();
  joint_effort_commanded_interfaces_.clear();
  joint_temperature_system_interfaces_.clear();
  joint_temperature_phase_a_interfaces_.clear();
  joint_temperature_phase_b_interfaces_.clear();
  joint_temperature_phase_c_interfaces_.clear();
  joint_bus_voltage_interfaces_.clear();

  // get the actual interface in an ordered way (same order as the joints parameter)
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints,
                                                    hardware_interface::HW_IF_POSITION, joint_position_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints,
                                                    hardware_interface::HW_IF_VELOCITY, joint_velocity_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - velocity");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, hardware_interface::HW_IF_EFFORT,
                                                    joint_effort_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - effort");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "position_commanded",
                                                    joint_position_commanded_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - position_commanded");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "velocity_commanded",
                                                    joint_velocity_commanded_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - velocity_commanded");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "effort_commanded",
                                                    joint_effort_commanded_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - effort_commanded");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "motor_temperature_system",
                                                    joint_temperature_system_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - motor_temperature_system");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "motor_temperature_coil_A",
                                                    joint_temperature_phase_a_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - motor_temperature_coil_A");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "motor_temperature_coil_B",
                                                    joint_temperature_phase_b_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - motor_temperature_coil_B");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "motor_temperature_coil_C",
                                                    joint_temperature_phase_c_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - motor_temperature_coil_C");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "motor_bus_voltage",
                                                    joint_bus_voltage_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered interfaces - motor_bus_voltage");
    return controller_interface::CallbackReturn::FAILURE;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
StatusBroadcaster::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type StatusBroadcaster::update(const rclcpp::Time& time,
                                                            [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  // create the status message for the whole arm
  ArmState state_msg;
  state_msg.header.stamp = time;
  state_msg.header.frame_id = params_.arm_name;

  // now for every joint we create a drive state message
  for (std::size_t i = 0; i < params_.joints.size(); i++) {
    DriveState drive_state_msg;

    drive_state_msg.name = params_.joints.at(i);
    drive_state_msg.joint_position = joint_position_interfaces_.at(i).get().get_value();
    drive_state_msg.joint_velocity = joint_velocity_interfaces_.at(i).get().get_value();
    drive_state_msg.joint_effort = joint_effort_interfaces_.at(i).get().get_value();
    drive_state_msg.temperature_system = joint_temperature_system_interfaces_.at(i).get().get_value();
    drive_state_msg.temperature_phase_a = joint_temperature_phase_a_interfaces_.at(i).get().get_value();
    drive_state_msg.temperature_phase_b = joint_temperature_phase_b_interfaces_.at(i).get().get_value();
    drive_state_msg.temperature_phase_c = joint_temperature_phase_c_interfaces_.at(i).get().get_value();
    drive_state_msg.bus_voltage = joint_bus_voltage_interfaces_.at(i).get().get_value();

    drive_state_msg.joint_position_commanded = joint_position_commanded_interfaces_.at(i).get().get_value();
    drive_state_msg.joint_velocity_commanded = joint_velocity_commanded_interfaces_.at(i).get().get_value();
    drive_state_msg.joint_effort_commanded = joint_effort_commanded_interfaces_.at(i).get().get_value();

    state_msg.states.emplace_back(drive_state_msg);
  }

  // and we try to have our realtime publisher publish the message
  // if this doesn't succeed - well it will probably next time
  if (arm_state_pub_rt_->trylock()) {
    arm_state_pub_rt_->msg_ = state_msg;
    arm_state_pub_rt_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}
}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::StatusBroadcaster, controller_interface::ControllerInterface)
