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

#include <dynaarm_controllers/freeze_controller.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace dynaarm_controllers
{
FreezeController::FreezeController()
{
}
controller_interface::InterfaceConfiguration FreezeController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(params_.arm_name + "/freeze_mode");
  return config;
}

controller_interface::InterfaceConfiguration FreezeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::CallbackReturn FreezeController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<freeze_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FreezeController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FreezeController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  freeze_mode_interface.clear();
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, std::vector<std::string>{ params_.arm_name },
                                                    "freeze_mode", freeze_mode_interface)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interface - freeze_mode");
    return controller_interface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Enable freeze mode for arm: " << params_.arm_name);
  if (!freeze_mode_interface.at(0).get().set_value(1.0)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Could not write freeze_mode field - this is a problem!");
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FreezeController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  if (params_.disable_at_deactivate) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Disable freeze mode for arm: " << params_.arm_name);
    if (!freeze_mode_interface.at(0).get().set_value(0.0)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Could not write freeze_mode field - this is a problem!");
    }
  } else {
    RCLCPP_WARN_STREAM(get_node()->get_logger(), "Do not disable freeze mode for arm: " << params_.arm_name
                                                                                        << " at deactivation - this "
                                                                                           "might lead to unexpected "
                                                                                           "behavior");
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FreezeController::update([[maybe_unused]] const rclcpp::Time& time,
                                                           [[maybe_unused]] const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}
}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::FreezeController, controller_interface::ControllerInterface)
