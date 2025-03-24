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

#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "dynaarm_driver/dynaarm_mock_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dynaarm_driver
{
hardware_interface::CallbackReturn
DynaarmMockHardwareInterface::on_init_derived(const hardware_interface::HardwareInfo& /*system_info*/)
{
  // Get base folder from hardware parameters
  const auto it = info_.hardware_parameters.find("drive_parameter_folder");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_WARN(logger_, "No drive_parameter_folder found in hardware parameters");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  const std::string base_directory = it->second + "/";

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    const auto& joint_name = info_.joints[i].name;
    const std::string yaml_path = base_directory + joint_name + ".yaml";
  
    if (!std::filesystem::exists(yaml_path)) {
      RCLCPP_WARN_STREAM(logger_, "No config found for joint: " << joint_name << " at " << yaml_path);
      continue;
    }
  
    YAML::Node config = YAML::LoadFile(yaml_path);
  
    if (!config["modes"]) {
      RCLCPP_WARN_STREAM(logger_, "Missing 'modes' section in config for joint: " << joint_name);
      continue;
    }
  
    bool found = false;
    for (const auto& mode : config["modes"]) {
      if (mode["name"] && mode["name"].as<std::string>() == "JointPositionVelocityTorquePidGains") {
        const auto& gains = mode["gains"];
        const double p = gains["p"].as<double>(0.0);
        const double i_ = gains["i"].as<double>(0.0);
        const double d = gains["d"].as<double>(0.0);
  
        joint_command_vector_[i].p_gain = p;
        joint_command_vector_[i].i_gain = i_;
        joint_command_vector_[i].d_gain = d;
  
        motor_command_vector_[i].p_gain = p;
        motor_command_vector_[i].i_gain = i_;
        motor_command_vector_[i].d_gain = d;
  
        RCLCPP_INFO_STREAM(logger_, "Loaded PID gains for " << joint_name
                                                            << " (JointPositionVelocityTorquePidGains): "
                                                            << "p=" << p << ", i=" << i_ << ", d=" << d);
        found = true;
        break;
      }
    }
  
    if (!found) {
      RCLCPP_WARN_STREAM(logger_, "No gains found for mode 'JointPositionVelocityTorquePidGains' in joint: " << joint_name);
    }
  }  

  RCLCPP_INFO_STREAM(logger_, "Successfully initialized dynaarm hardware interface for DynaarmMockHardwareInterface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynaarmMockHardwareInterface::on_activate_derived(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO_STREAM(logger_, "Successfully activated dynaarm hardware interface for DynaarmMockHardwareInterface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynaarmMockHardwareInterface::on_deactivate_derived(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO_STREAM(logger_, "Successfully deactivated dynaarm hardware interface for DynaarmMockHardwareInterface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void DynaarmMockHardwareInterface::read_motor_states()
{
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    if (!frozen_joint_positions_.empty()) {
      motor_state_vector_[i].position = frozen_joint_positions_[i];
      motor_state_vector_[i].velocity = 0.0;
      motor_state_vector_[i].effort = 0.0;
    } else {
      motor_state_vector_[i].position = motor_command_vector_[i].position;
      motor_state_vector_[i].velocity = motor_command_vector_[i].velocity;
      motor_state_vector_[i].effort = motor_command_vector_[i].effort;
    }

    motor_state_vector_[i].temperature = 0.0;
    motor_state_vector_[i].temperature_coil_A = 0.0;
    motor_state_vector_[i].temperature_coil_B = 0.0;
    motor_state_vector_[i].temperature_coil_C = 0.0;
    motor_state_vector_[i].bus_voltage = 0.0;
  }
}

void DynaarmMockHardwareInterface::write_motor_commands()
{
  if (command_freeze_mode_ == 1.0) {
    if (frozen_joint_positions_.empty()) {
      frozen_joint_positions_.resize(info_.joints.size());
      for (std::size_t i = 0; i < info_.joints.size(); i++) {
        frozen_joint_positions_[i] = motor_state_vector_[i].position;
      }
      RCLCPP_INFO(logger_, "Freeze mode activated — holding current joint positions");
    }  
    
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      // Override any drifted command with the frozen position
      motor_command_vector_[i].position = frozen_joint_positions_[i];
      motor_command_vector_[i].velocity = 0.0;
      motor_command_vector_[i].effort = 0.0;
    }
    return;     
  }
  
  if (!frozen_joint_positions_.empty()) {
    frozen_joint_positions_.clear();
    RCLCPP_INFO(logger_, "Freeze mode deactivated — resuming joint updates");  

    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      // Override any drifted command with the frozen position
      motor_command_vector_[i].position = frozen_joint_positions_[i];
      motor_command_vector_[i].velocity = 0.0;
      motor_command_vector_[i].effort = 0.0;
    }
    return;  // skip this update to block any immediate overwrite
  }  

  // Normal behavior: command becomes state
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    motor_state_vector_[i].position = motor_command_vector_[i].position;
    motor_state_vector_[i].velocity = motor_command_vector_[i].velocity;
    motor_state_vector_[i].effort = motor_command_vector_[i].effort;
    motor_command_vector_[i].p_gain = joint_command_vector_[i].p_gain;
    motor_command_vector_[i].i_gain = joint_command_vector_[i].i_gain;
    motor_command_vector_[i].d_gain = joint_command_vector_[i].d_gain;
  }
}

DynaarmMockHardwareInterface::~DynaarmMockHardwareInterface()
{
  RCLCPP_INFO_STREAM(logger_, "Destroy DynaarmMockHardwareInterface");
}

}  // namespace dynaarm_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_driver::DynaarmMockHardwareInterface, hardware_interface::SystemInterface)
