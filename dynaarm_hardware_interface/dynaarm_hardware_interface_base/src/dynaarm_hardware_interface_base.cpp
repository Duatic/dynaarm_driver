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

#include "dynaarm_hardware_interface_base/dynaarm_hardware_interface_base.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dynaarm_hardware_interface_base
{
DynaArmHardwareInterfaceBase::~DynaArmHardwareInterfaceBase()
{
  safeShutdown();
}

void DynaArmHardwareInterfaceBase::safeShutdown()
{
  try
  {
    shutdown();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "Exception during safe shutdown: %s", e.what());
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Unknown exception during safe shutdown");
  }
}

hardware_interface::CallbackReturn
DynaArmHardwareInterfaceBase::on_init(const hardware_interface::HardwareInfo& system_info)
{
  // Init base interface
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_FATAL(logger_, "Error initialising base interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Store the system information
  info_ = system_info;

  // Initialize the state vectors with 0 values
  for (int i = 0; i < static_cast<int>(info_.joints.size()); i++)
  {
    joint_state_vector_.push_back(dynaarm_hardware_interface_common::JointState{ .name = info_.joints[i].name });
    joint_command_vector_.push_back(dynaarm_hardware_interface_common::JointCommand{ .name = info_.joints[i].name });
    motor_state_vector_.push_back(dynaarm_hardware_interface_common::MotorState{ .name = info_.joints[i].name });
    motor_command_vector_.push_back(dynaarm_hardware_interface_common::MotorCommand{ .name = info_.joints[i].name });
  }

  RCLCPP_INFO_STREAM(logger_, "Successfully initialized dynaarm hardware interface for DynaArmHardwareInterfaceBase");

  return on_init_derived(system_info);
}

std::vector<hardware_interface::StateInterface> DynaArmHardwareInterfaceBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto& joint_state : joint_state_vector_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_state.name, hardware_interface::HW_IF_POSITION, &joint_state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_state.name, hardware_interface::HW_IF_VELOCITY, &joint_state.velocity));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_state.name, hardware_interface::HW_IF_EFFORT, &joint_state.effort));
  }

  for (auto& joint_command : joint_command_vector_)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "position_commanded", &joint_command.position));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "velocity_commanded", &joint_command.velocity));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "effort_commanded", &joint_command.effort));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "p_gain_commanded", &joint_command.p_gain));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "i_gain_commanded", &joint_command.i_gain));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "d_gain_commanded", &joint_command.d_gain));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_command.name, "command_freeze_mode",
                                                                     &joint_command.command_freeze_mode));
  }

  for (auto& motor_state : motor_state_vector_)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_state.name, "motor_position", &motor_state.position));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_state.name, "motor_velocity", &motor_state.velocity));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_state.name, "motor_effort", &motor_state.effort));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_state.name, "motor_temperature_system", &motor_state.temperature));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor_state.name, "motor_temperature_coil_A",
                                                                     &motor_state.temperature_coil_A));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor_state.name, "motor_temperature_coil_B",
                                                                     &motor_state.temperature_coil_B));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor_state.name, "motor_temperature_coil_C",
                                                                     &motor_state.temperature_coil_C));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_state.name, "motor_bus_voltage", &motor_state.bus_voltage));
  }

  for (auto& motor_command : motor_command_vector_)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_command.name, "motor_position_commanded", &motor_command.position));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_command.name, "motor_velocity_commanded", &motor_command.velocity));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_command.name, "motor_effort_commanded", &motor_command.effort));
  }

  // TODO expose drive state, warnings, imu?
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynaArmHardwareInterfaceBase::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto& joint_command : joint_command_vector_)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_command.name, hardware_interface::HW_IF_POSITION, &joint_command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_command.name, hardware_interface::HW_IF_VELOCITY, &joint_command.velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_command.name, hardware_interface::HW_IF_EFFORT, &joint_command.effort));

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_command.name, "p_gain", &joint_command.p_gain));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_command.name, "i_gain", &joint_command.i_gain));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_command.name, "d_gain", &joint_command.d_gain));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_command.name, "command_freeze_mode",
                                                                         &joint_command.command_freeze_mode));
  }
  // Expose a dummy command interface for the freeze mode

  return command_interfaces;
}

hardware_interface::CallbackReturn
DynaArmHardwareInterfaceBase::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  hardware_interface::CallbackReturn callbackReturn = on_activate_derived(previous_state);

  // Perform a reading once to obtain the current positions
  read(rclcpp::Time(), rclcpp::Duration(std::chrono::nanoseconds(0)));

  for (int i = 0; i < static_cast<int>(info_.joints.size()); i++)
  {
    joint_command_vector_[i].position = joint_state_vector_[i].position;
    joint_command_vector_[i].velocity = 0.0;
    joint_command_vector_[i].effort = 0.0;
    RCLCPP_INFO_STREAM(logger_, "Start position of joint: " << info_.joints[i].name
                                                            << " is: " << joint_state_vector_[i].position);
  }

  return callbackReturn;
}

hardware_interface::CallbackReturn
DynaArmHardwareInterfaceBase::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return on_deactivate_derived(previous_state);
}

hardware_interface::CallbackReturn
DynaArmHardwareInterfaceBase::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
  safeShutdown();
  return hardware_interface::CallbackReturn::FAILURE;
}

hardware_interface::return_type DynaArmHardwareInterfaceBase::read(const rclcpp::Time& /*time*/,
                                                                   const rclcpp::Duration& /*period*/)
{
  // updates the motor states. Assumed that after this function the motor_state_vector is correctly updated
  read_motor_states();

  Eigen::VectorXd motor_position(info_.joints.size());
  Eigen::VectorXd motor_velocity(info_.joints.size());
  Eigen::VectorXd motor_effort(info_.joints.size());

  for (int i = 0; i < static_cast<int>(info_.joints.size()); i++)
  {
    motor_position(i) = motor_state_vector_[i].position;
    motor_velocity(i) = motor_state_vector_[i].velocity;
    motor_effort(i) = motor_state_vector_[i].effort;
  }
  // Transform the joint positions using the matrix transformation
  Eigen::VectorXd joint_position =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialCoordinates(motor_position);
  Eigen::VectorXd joint_velocity =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialCoordinates(motor_velocity);
  Eigen::VectorXd joint_effort =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialTorques(motor_effort);

  for (int i = 0; i < static_cast<int>(info_.joints.size()); ++i)
  {
    joint_state_vector_[i].position = joint_position[i];
    joint_state_vector_[i].velocity = joint_velocity[i];
    joint_state_vector_[i].effort = joint_effort[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynaArmHardwareInterfaceBase::write(const rclcpp::Time& /*time*/,
                                                                    const rclcpp::Duration& /*period*/)
{
  Eigen::VectorXd joint_position(info_.joints.size());
  Eigen::VectorXd joint_velocity(info_.joints.size());
  Eigen::VectorXd joint_effort(info_.joints.size());
  for (int i = 0; i < static_cast<int>(info_.joints.size()); ++i)
  {
    joint_position[i] = joint_command_vector_[i].position;
    joint_velocity[i] = joint_command_vector_[i].velocity;
    joint_effort[i] = joint_command_vector_[i].effort;
  }

  // Transform the joint positions using the matrix transformation
  Eigen::VectorXd motor_position =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromSerialToDynaarmCoordinates(joint_position);
  Eigen::VectorXd motor_velocity =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromSerialToDynaarmCoordinates(joint_velocity);
  Eigen::VectorXd motor_effort =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromSerialToDynaarmTorques(joint_effort);

  for (int i = 0; i < static_cast<int>(info_.joints.size()); ++i)
  {
    motor_command_vector_[i].position = motor_position[i];
    motor_command_vector_[i].velocity = motor_velocity[i];
    motor_command_vector_[i].effort = motor_effort[i];
    motor_command_vector_[i].p_gain = joint_command_vector_[i].p_gain;
    motor_command_vector_[i].i_gain = joint_command_vector_[i].i_gain;
    motor_command_vector_[i].d_gain = joint_command_vector_[i].d_gain;
    motor_command_vector_[i].command_freeze_mode = joint_command_vector_[i].command_freeze_mode;
  }

  // writes motor commands to the drives, simulation or directly into motor_state for mock
  write_motor_commands();

  return hardware_interface::return_type::OK;
}
}  // namespace dynaarm_hardware_interface_base
