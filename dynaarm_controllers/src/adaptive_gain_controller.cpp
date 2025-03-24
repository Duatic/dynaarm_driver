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

#include <dynaarm_controllers/adaptive_gain_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace dynaarm_controllers
{

AdaptiveGainController::AdaptiveGainController()
{
}

controller_interface::InterfaceConfiguration AdaptiveGainController::command_interface_configuration() const
{
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

controller_interface::InterfaceConfiguration AdaptiveGainController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  return config;
}

controller_interface::CallbackReturn AdaptiveGainController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<adaptive_gain_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveGainController::on_configure(const rclcpp_lifecycle::State&)
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

  pinocchio::urdf::buildModelFromXML(get_robot_description(), pinocchio_model_);
  pinocchio_data_ = pinocchio::Data(pinocchio_model_);

  // Extract joint names from Pinocchio model
  std::vector<std::string> pinocchio_joint_names;
  for (size_t i = 1; i < pinocchio_model_.joints.size(); ++i)  // Start from 1 to skip the universe/root joint
  {
    std::string joint_name = pinocchio_model_.names[i];
    pinocchio_joint_names.push_back(joint_name);
  }

  // 1. Validate joint names (amount)
  if (pinocchio_joint_names.size() != params_.joints.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Joint count mismatch: Pinocchio model has %zu joints, but interface has %zu joints.",
                 pinocchio_joint_names.size(), params_.joints.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 2. Validate joint names order
  for (size_t i = 0; i < pinocchio_joint_names.size(); ++i) {
    if (pinocchio_joint_names[i] != params_.joints[i]) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint name mismatch at index %zu: Pinocchio joint is '%s', interface joint is '%s'.", i,
                   pinocchio_joint_names[i].c_str(), params_.joints[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveGainController::on_activate(const rclcpp_lifecycle::State&)
{
  active_ = true;

  joint_p_gain_command_interfaces_.clear();
  joint_i_gain_command_interfaces_.clear();
  joint_d_gain_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();

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

  // 🆕 Read initial gains from hardware
  base_p_gains_.clear();
  base_i_gains_.clear();
  base_d_gains_.clear();

  for (std::size_t i = 0; i < params_.joints.size(); ++i) {
    const double p = joint_p_gain_command_interfaces_[i].get().get_value();
    const double i_ = joint_i_gain_command_interfaces_[i].get().get_value();
    const double d = joint_d_gain_command_interfaces_[i].get().get_value();

    base_p_gains_.push_back(p);
    base_i_gains_.push_back(i_);
    base_d_gains_.push_back(d);

    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Base gains [" << params_.joints[i] << "]: P=" << p << ", I=" << i_ << ", D=" << d);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveGainController::on_deactivate(const rclcpp_lifecycle::State&)
{
  if (!active_) {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  const std::size_t joint_count = joint_p_gain_command_interfaces_.size();
  for (std::size_t i = 0; i < joint_count; i++) {
    const double p = base_p_gains_[i];
    const double i_ = base_i_gains_[i];
    const double d = base_d_gains_[i];

    (void)joint_p_gain_command_interfaces_[i].get().set_value(p);
    (void)joint_i_gain_command_interfaces_[i].get().set_value(i_);
    (void)joint_d_gain_command_interfaces_[i].get().set_value(d);

    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Restored base gains [" << params_.joints[i] << "]: P=" << p << ", I=" << i_ << ", D=" << d);
  }

  active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdaptiveGainController::update(const rclcpp::Time&, const rclcpp::Duration&)
{
  if (!active_) {
    return controller_interface::return_type::OK;
  }

  Eigen::VectorXd q(joint_position_state_interfaces_.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    q[i] = joint_position_state_interfaces_[i].get().get_value();
  }

  pinocchio::crba(pinocchio_model_, pinocchio_data_, q);
  pinocchio_data_.M.triangularView<Eigen::StrictlyLower>() =
      pinocchio_data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  const Eigen::VectorXd diag_inertia = pinocchio_data_.M.diagonal();

  for (std::size_t i = 0; i < params_.joints.size(); ++i) {
    const double scale = std::clamp(1.0 / diag_inertia[i], 0.2, 2.0);  // You can fine-tune this logic

    const double p_scaled = base_p_gains_[i] * scale;
    const double i_scaled = base_i_gains_[i] * scale;
    const double d_scaled = base_d_gains_[i] * scale;

    joint_p_gain_command_interfaces_[i].get().set_value(p_scaled);
    joint_i_gain_command_interfaces_[i].get().set_value(i_scaled);
    joint_d_gain_command_interfaces_[i].get().set_value(d_scaled);

    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Adaptive gains [" << params_.joints[i] << "]: P=" << p_scaled
                                                                     << ", I=" << i_scaled << ", D=" << d_scaled
                                                                     << " (scale=" << scale << ")");
  }

  return controller_interface::return_type::OK;
}

}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::AdaptiveGainController, controller_interface::ControllerInterface)
