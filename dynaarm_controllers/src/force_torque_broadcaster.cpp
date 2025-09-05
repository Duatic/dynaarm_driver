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

#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

#include <dynaarm_controllers/force_torque_broadcaster.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace dynaarm_controllers
{

ForceTorqueBroadcaster::ForceTorqueBroadcaster() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration ForceTorqueBroadcaster::command_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::InterfaceConfiguration ForceTorqueBroadcaster::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    config.names.emplace_back(joint + "/" + "acceleration_commanded");
  }

  return config;
}

controller_interface::CallbackReturn ForceTorqueBroadcaster::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<force_torque_broadcaster::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
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

  // Extract joint names from Pinocchio model that match params_.joints
  std::vector<std::string> pinocchio_joint_names;
  for (size_t i = 1; i < pinocchio_model_.joints.size(); ++i)  // Start from 1 to skip the universe/root joint
  {
    std::string joint_name = pinocchio_model_.names[i];
    // Only add if this joint is in params_.joints
    if (std::find(params_.joints.begin(), params_.joints.end(), joint_name) != params_.joints.end()) {
      pinocchio_joint_names.push_back(joint_name);
    }
  }

  // 1. Validate joint names (amount)
  if (pinocchio_joint_names.size() != params_.joints.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Joint count mismatch: Pinocchio model has %zu relevant joints, but interface has %zu joints.",
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

  frame_index_ = pinocchio_model_.getFrameId(params_.endeffector_frame);

  // The wrench publisher
  wrench_pub_ =
      get_node()->create_publisher<WrenchStamped>("~/wrench", 10);  // TODO(firesurfer) what is the right qos ?
  wrench_pub_rt_ = std::make_unique<WrenchStampedPublisher>(wrench_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // clear out vectors in case of restart
  joint_effort_state_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();

  // get the actual interface in an ordered way (same order as the joints parameter)
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, hardware_interface::HW_IF_POSITION, joint_position_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, hardware_interface::HW_IF_VELOCITY, joint_velocity_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - velocity");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "acceleration_commanded",
                                                    joint_acceleration_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - acceleration");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, hardware_interface::HW_IF_EFFORT,
                                                    joint_effort_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - effort");
    return controller_interface::CallbackReturn::FAILURE;
  }

  active_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceTorqueBroadcaster::update([[maybe_unused]] const rclcpp::Time& time,
                                                                 [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE || !active_) {
    return controller_interface::return_type::OK;
  }

  const std::size_t joint_count = joint_position_state_interfaces_.size();

  // Build full-size vectors for all robot joints (Pinocchio expects this)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  Eigen::VectorXd torque_meas = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  // Map: Pinocchio joint name -> index in q/v
  for (std::size_t i = 0; i < joint_count; i++) {
    const std::string& joint_name = params_.joints[i];
    auto idx = pinocchio_model_.getJointId(joint_name);
    if (idx == 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
      return controller_interface::return_type::ERROR;
    }
    // Pinocchio joint index starts at 1, q/v index is idx-1
    q[pinocchio_model_.joints[idx].idx_q()] = joint_position_state_interfaces_.at(i).get().get_value();
    v[pinocchio_model_.joints[idx].idx_v()] = joint_velocity_state_interfaces_.at(i).get().get_value();
    a[pinocchio_model_.joints[idx].idx_v()] = joint_acceleration_state_interfaces_.at(i).get().get_value();
    torque_meas[pinocchio_model_.joints[idx].idx_v()] = joint_effort_state_interfaces_.at(i).get().get_value();
  }

  forwardKinematics(pinocchio_model_, pinocchio_data_, q, v, a);
  const auto tau = pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, v, a);

  Eigen::VectorXd tau_ext = tau - torque_meas;

  pinocchio::computeJointJacobians(pinocchio_model_, pinocchio_data_, q);  // Or LOCAL
  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, pinocchio_model_.nv);
  pinocchio::getFrameJacobian(pinocchio_model_, pinocchio_data_, frame_index_, pinocchio::LOCAL_WORLD_ALIGNED,
                              J);  // LOCAL or LOCAL_WORLD_ALIGNED

  // Solve for wrench: F = (J^T)^+ * tau_ext
  Eigen::VectorXd wrench = J.transpose().completeOrthogonalDecomposition().solve(tau_ext);

  WrenchStamped wrench_msg;
  wrench_msg.header.stamp = time;
  wrench_msg.header.frame_id = params_.endeffector_frame;

  // Force
  wrench_msg.wrench.force.x = wrench(0);
  wrench_msg.wrench.force.y = wrench(1);
  wrench_msg.wrench.force.z = wrench(2);

  // Torque
  wrench_msg.wrench.torque.x = wrench(3);
  wrench_msg.wrench.torque.y = wrench(4);
  wrench_msg.wrench.torque.z = wrench(5);
  // Write only the efforts for this arm's joints
  for (std::size_t i = 0; i < joint_count; i++) {
    const std::string& joint_name = params_.joints[i];
    auto idx = pinocchio_model_.getJointId(joint_name);
  }
  // and we try to have our realtime publisher publish the message
  // if this doesn't succeed - well it will probably next time
  if (wrench_pub_rt_->trylock()) {
    wrench_pub_rt_->msg_ = wrench_msg;
    wrench_pub_rt_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_error([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::ForceTorqueBroadcaster, controller_interface::ControllerInterface)
