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

#pragma once

// System
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

// ros2_control hardware_interface
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

// ROS
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

// Urdf
#include "dynaarm_hardware_interface_base/types.hpp"
#include "dynaarm_hardware_interface_base/command_translator.hpp"

namespace dynaarm_hardware_interface_base
{
class DynaArmHardwareInterfaceBase : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynaArmHardwareInterfaceBase)

  DynaArmHardwareInterfaceBase() : logger_(rclcpp::get_logger("DynaArmHardwareInterfaceBase"))
  {
    // This is only here otherwise the compiler will complain about the logger var.
    // We initialize the logger in on_init properly
  }
  virtual ~DynaArmHardwareInterfaceBase();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& system_info);
  virtual hardware_interface::CallbackReturn on_init_derived(const hardware_interface::HardwareInfo& system_info) = 0;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
  virtual hardware_interface::CallbackReturn on_activate_derived(const rclcpp_lifecycle::State& previous_state) = 0;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
  virtual hardware_interface::CallbackReturn on_deactivate_derived(const rclcpp_lifecycle::State& previous_state) = 0;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period);
  virtual void read_motor_states() = 0;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period);
  virtual void write_motor_commands() = 0;

  CallbackReturn on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state) override
  {
    RCLCPP_INFO_STREAM(logger_, "Base on_configure");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces);

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces);

protected:
  rclcpp::Logger logger_;

  std::vector<dynaarm_hardware_interface_common::JointState> joint_state_vector_;
  std::vector<dynaarm_hardware_interface_common::MotorState> motor_state_vector_;

  std::vector<dynaarm_hardware_interface_common::JointCommand> joint_command_vector_;
  std::vector<dynaarm_hardware_interface_common::MotorCommand> motor_command_vector_;

  double command_freeze_mode_{ 0.0 };  // do not start in freeze mode per default -> We allow overriding this via a
                                       // ros2control parameters

  std::atomic<bool> active_{ false };
};

}  // namespace dynaarm_hardware_interface_base
