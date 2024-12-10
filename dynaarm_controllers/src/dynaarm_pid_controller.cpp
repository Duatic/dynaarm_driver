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

#include "dynaarm_controllers/dynaarm_pid_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace dynaarm_controllers
{
PIDController::PIDController()
{
}
controller_interface::InterfaceConfiguration PIDController::command_interface_configuration() const
{
  // Claim the necessary command interfaces

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + "p_gain");
    config.names.emplace_back(joint + "/" + "i_gain");
    config.names.emplace_back(joint + "/" + "d_gain");
  }
  return config;
}

controller_interface::InterfaceConfiguration PIDController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::CallbackReturn PIDController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<dynaarm_pid_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PIDController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
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

  for (const auto& joint : params_.joints) {
    // TODO(firesurfer) this is the easiest way to implement this but also the one with the lowest performance
    // (parameter calls are quite expensive)
    gain_subscriptions_.push_back(get_node()->create_subscription<PIDGains>(
        "~/" + joint + "/pid_gains/set", 10, [joint, this](const PIDGains& msg) {
          const std::string param_name_base = joint + "/";
          const std::string p_gain_param = param_name_base + "p_gain";
          const std::string i_gain_param = param_name_base + "i_gain";
          const std::string d_gain_param = param_name_base + "d_gain";

          get_node()->set_parameter(rclcpp::Parameter(p_gain_param, msg.p));
          get_node()->set_parameter(rclcpp::Parameter(i_gain_param, msg.i));
          get_node()->set_parameter(rclcpp::Parameter(d_gain_param, msg.d));
        }));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PIDController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  joint_p_gain_command_interfaces_.clear();
  joint_i_gain_command_interfaces_.clear();
  joint_d_gain_command_interfaces_.clear();

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

  // Declare a parameter for each pid gain for each joint
  const auto joint_size = params_.joints.size();
  auto&& node = get_node();
  for (std::size_t i = 0; i < joint_size; i++) {
    const std::string param_name_base = params_.joints[i] + "/";
    const std::string p_gain_param = param_name_base + "p_gain";
    const std::string i_gain_param = param_name_base + "i_gain";
    const std::string d_gain_param = param_name_base + "d_gain";

    auto set_param = [&node](const std::string& name, const CommandInterfaceReference& interface) {
      // Either declare or update the parameter
      if (!node->has_parameter(name)) {
        node->declare_parameter(name, interface.get().get_value());
      } else {
        node->set_parameter(rclcpp::Parameter(name, interface.get().get_value()));
      }
    };

    set_param(p_gain_param, joint_p_gain_command_interfaces_[i]);
    set_param(i_gain_param, joint_i_gain_command_interfaces_[i]);
    set_param(d_gain_param, joint_d_gain_command_interfaces_[i]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PIDController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PIDController::update([[maybe_unused]] const rclcpp::Time& time,
                                                        [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  const auto joint_size = params_.joints.size();
  auto&& node = get_node();
  for (std::size_t i = 0; i < joint_size; i++) {
    // Build the param names
    const std::string param_name_base = params_.joints[i] + "/";
    const std::string p_gain_param = param_name_base + "p_gain";
    const std::string i_gain_param = param_name_base + "i_gain";
    const std::string d_gain_param = param_name_base + "d_gain";

    // Obtain the target gains and limit them
    const double p_gain = limit_gain(node->get_parameter(p_gain_param).as_double());
    const double i_gain = limit_gain(node->get_parameter(i_gain_param).as_double());
    const double d_gain = limit_gain(node->get_parameter(d_gain_param).as_double());

    // Set the target gains
    (void)joint_p_gain_command_interfaces_[i].get().set_value(p_gain);
    (void)joint_i_gain_command_interfaces_[i].get().set_value(i_gain);
    (void)joint_d_gain_command_interfaces_[i].get().set_value(d_gain);
  }

  return controller_interface::return_type::OK;
}
}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::PIDController, controller_interface::ControllerInterface)
