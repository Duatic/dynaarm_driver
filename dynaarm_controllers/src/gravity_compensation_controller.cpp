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

#include "dynaarm_controllers/gravity_compensation_controller.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using config_type = controller_interface::interface_configuration_type;

namespace dynaarm_controllers
{

GravityCompensationController::GravityCompensationController() : controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  std::string urdf_string = get_robot_description();

  pinocchio::urdf::buildModelFromXML(urdf_string, model);
  data = pinocchio::Data(model);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = { config_type::INDIVIDUAL, {} };
  conf.names.reserve(joint_names_.size() * command_interface_types_.size());

  for (const auto& joint_name : joint_names_) {
    for (const auto& interface_type : command_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = { config_type::INDIVIDUAL, {} };
  conf.names.reserve(joint_names_.size() * state_interface_types_.size());

  for (const auto& joint_name : joint_names_) {
    for (const auto& interface_type : state_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn
GravityCompensationController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                        [[maybe_unused]] const rclcpp::Duration& period)
{
  //
  // THE OLD VERSION WHICH ITERATED OVER ALL JOINTS
  //
  // Eigen::VectorXd q = pinocchio::neutral(model);
  // // convert ROS joint config to pinocchio config
  // for (int i = 0; i < model.nv; i++)
  // {
  //   int jidx = model.getJointId(model.names[i + 1]);
  //   int qidx = model.idx_qs[jidx];
  //   q[qidx] = joint_position_state_interface_[i].get().get_value();
  // }

  // Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model, data, q);
  // std::cout << std::fixed << std::setprecision(10);

  // // add gravity compensation torque to base command
  // for (int i = 0; i < model.nv; i++)
  // {
  //   double new_effort = gravity[i] * 1.0;

  //   if (i == 1)
  //   {
  //     // std::cout << i << " - NEW: " << new_effort << std::endl;
  //     joint_effort_command_interface_[i].get().set_value(new_effort);
  //   }
  // }

  // Set up pinocchio configuration
  Eigen::VectorXd q = pinocchio::neutral(model);
  int jidx = model.getJointId(model.names[2]);  // Directly access joint ID for i == 1 (+1)
  int qidx = model.idx_qs[jidx];
  q[qidx] = joint_position_state_interface_[1].get().get_value();

  // Compute gravity compensation torque for the configuration
  Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model, data, q);

  // Apply the gravity compensation torque directly for joint i == 1
  double new_effort = gravity[1] * 1.0;
  joint_effort_command_interface_[1].get().set_value(new_effort);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
GravityCompensationController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_effort_command_interface_.clear();
  joint_p_command_interface_.clear();
  joint_i_command_interface_.clear();
  joint_d_command_interface_.clear();

  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  joint_effort_state_interface_.clear();

  // assign command interfaces
  for (auto& interface : command_interfaces_) {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for (auto& interface : state_interfaces_) {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCompensationController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCompensationController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCompensationController::on_error([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCompensationController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}
}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::GravityCompensationController, controller_interface::ControllerInterface)
