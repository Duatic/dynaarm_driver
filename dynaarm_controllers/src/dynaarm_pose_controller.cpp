/*
 * Copyright 2025 Duatic AG
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

#include <dynaarm_controllers/dynaarm_pose_controller.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace dynaarm_controllers
{

DynaArmPoseController::DynaArmPoseController()
{
}

controller_interface::InterfaceConfiguration DynaArmPoseController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
};

controller_interface::InterfaceConfiguration DynaArmPoseController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
};

controller_interface::CallbackReturn DynaArmPoseController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<dynaarm_pose_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
DynaArmPoseController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  activate_pose_client_ = get_node()->create_client<std_srvs::srv::SetBool>("activate_pose_controller");

  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
DynaArmPoseController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = true;
  call_activate_pose_controller_service(true);
  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
DynaArmPoseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = false;
  call_activate_pose_controller_service(false);
  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::return_type DynaArmPoseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                [[maybe_unused]] const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
};

void DynaArmPoseController::call_activate_pose_controller_service(bool activate)
{
  if (!activate_pose_client_) {
    RCLCPP_ERROR(get_node()->get_logger(), "activate_pose_controller client not initialized");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = activate;
  if (!activate_pose_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Service activate_pose_controller not available");
    return;
  }
  auto result = activate_pose_client_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        if (future.valid() && future.get()->success) {
          RCLCPP_INFO(get_node()->get_logger(), "Service call succeeded: %s", future.get()->message.c_str());
        } else {
          RCLCPP_ERROR(get_node()->get_logger(), "Service call failed or returned false");
        }
      });
}

}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::DynaArmPoseController, controller_interface::ControllerInterface)
