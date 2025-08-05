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

#include <dynaarm_controllers/freedrive_controller.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace dynaarm_controllers
{
FreeDriveController::FreeDriveController()
{
}
controller_interface::InterfaceConfiguration FreeDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + "p_gain");
    config.names.emplace_back(joint + "/" + "i_gain");
    config.names.emplace_back(joint + "/" + "d_gain");
  }
  return config;
};

controller_interface::InterfaceConfiguration FreeDriveController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
};

controller_interface::CallbackReturn FreeDriveController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<freedrive_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
FreeDriveController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
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
  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
FreeDriveController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = true;

  // clear out vectors in case of restart
  joint_position_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();

  joint_p_gain_command_interfaces_.clear();
  joint_i_gain_command_interfaces_.clear();
  joint_d_gain_command_interfaces_.clear();

  // get the actual interface in an ordered way (same order as the joints parameter)
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints,
                                                    hardware_interface::HW_IF_POSITION,
                                                    joint_position_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, "p_gain",
                                                    joint_p_gain_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - p_gain");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, "i_gain",
                                                    joint_i_gain_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - i_gain");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, "d_gain",
                                                    joint_d_gain_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - d_gain");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, hardware_interface::HW_IF_POSITION, joint_position_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Store previous gains
  previous_gains_.clear();
  const std::size_t joint_count = joint_position_state_interfaces_.size();
  for (std::size_t i = 0; i < joint_count; i++) {
    Gains g;
    g.p = joint_p_gain_command_interfaces_[i].get().get_value();
    g.i = joint_i_gain_command_interfaces_[i].get().get_value();
    g.d = joint_d_gain_command_interfaces_[i].get().get_value();
    previous_gains_.emplace_back(g);

    RCLCPP_DEBUG_STREAM(get_node()->get_logger(),
                        "Previous gains: " << params_.joints[i] << " p: " << g.p << " i: " << g.i << " d: " << g.d);
  }

  // Manipulate gains
  for (std::size_t i = 0; i < joint_count; i++) {
    // We disable the position tracking
    if (!joint_p_gain_command_interfaces_[i].get().set_value(0.0)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), params_.joints[i] << ": error setting p-gain");
    }
    if (!joint_i_gain_command_interfaces_[i].get().set_value(0.0)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), params_.joints[i] << ": error setting i-gain");
    }

    auto d_gain_value = params_.d_gains[i];
    if (!joint_d_gain_command_interfaces_[i].get().set_value(d_gain_value)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), params_.joints[i] << ": error setting d-gain");
    }

    RCLCPP_DEBUG_STREAM(get_node()->get_logger(),
                        "Set gains: " << params_.joints[i] << " p: " << 0.0 << " i: " << 0.0 << " d: " << d_gain_value);
  }

  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
FreeDriveController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // Restore controller gains

  const std::size_t joint_count = joint_position_state_interfaces_.size();
  for (std::size_t i = 0; i < joint_count; i++) {
    const auto& g = previous_gains_[i];
    (void)joint_p_gain_command_interfaces_[i].get().set_value(g.p);
    (void)joint_i_gain_command_interfaces_[i].get().set_value(g.i);
    (void)joint_d_gain_command_interfaces_[i].get().set_value(g.d);
    RCLCPP_DEBUG_STREAM(get_node()->get_logger(),
                        "Restore gains: " << params_.joints[i] << " p: " << g.p << " i: " << g.i << " d: " << g.d);
  }

  active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::return_type FreeDriveController::update([[maybe_unused]] const rclcpp::Time& time,
                                                              [[maybe_unused]] const rclcpp::Duration& period)
{
  const std::size_t joint_count = joint_position_state_interfaces_.size();

  // Never the less we command the current joint position. This is only important when switching from this controller to
  // another one
  for (std::size_t i = 0; i < joint_count; i++) {
    const double current_joint_position = joint_position_state_interfaces_.at(i).get().get_value();
    bool success = joint_position_command_interfaces_.at(i).get().set_value(current_joint_position);

    if (!success) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Error wring value to command interface: "
                                                        << joint_position_command_interfaces_.at(i).get().get_name());
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
};

}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::FreeDriveController, controller_interface::ControllerInterface)
