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

#include "dynaarm_driver/dynaarm_mock_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dynaarm_driver
{
hardware_interface::CallbackReturn
DynaarmMockHardwareInterface::on_init_derived(const hardware_interface::HardwareInfo& /*system_info*/)
{
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
    motor_state_vector_[i].position = motor_command_vector_[i].position;
    motor_state_vector_[i].velocity = motor_command_vector_[i].velocity;
    motor_state_vector_[i].effort = motor_command_vector_[i].effort;

    motor_state_vector_[i].temperature = 0.0;
    motor_state_vector_[i].temperature_coil_A = 0.0;
    motor_state_vector_[i].temperature_coil_B = 0.0;
    motor_state_vector_[i].temperature_coil_C = 0.0;
    motor_state_vector_[i].bus_voltage = 0.0;
  }
}

void DynaarmMockHardwareInterface::write_motor_commands()
{
}

DynaarmMockHardwareInterface::~DynaarmMockHardwareInterface()
{
  RCLCPP_INFO_STREAM(logger_, "Destroy DynaarmMockHardwareInterface");
}

}  // namespace dynaarm_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_driver::DynaarmMockHardwareInterface, hardware_interface::SystemInterface)
