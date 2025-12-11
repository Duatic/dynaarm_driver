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

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "dynaarm_hardware_interface_base/dynaarm_hardware_interface_base.hpp"
#include "dynaarm_hardware_interface_base/string_utils.hpp"

namespace dynaarm_hardware_interface_base
{
DynaArmHardwareInterfaceBase::~DynaArmHardwareInterfaceBase()
{
}

hardware_interface::CallbackReturn
DynaArmHardwareInterfaceBase::on_init(const hardware_interface::HardwareComponentInterfaceParams& system_info)
{
  // Init base interface
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_FATAL(logger_, "Error initialising base interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check the amount of joints. We only support 4 or 6
  if (info_.joints.size() != 6 && info_.joints.size() != 4) {
    RCLCPP_FATAL_STREAM(logger_, "We need either 4 or 6 DoF but: " << info_.joints.size() << " were configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize the state vectors with 0 values
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    const auto joint_name = info_.joints.at(i).name;

    joint_state_vector_.push_back(dynaarm_hardware_interface_common::JointState{ .name = joint_name });
    joint_command_vector_.push_back(dynaarm_hardware_interface_common::JointCommand{ .name = joint_name });
    motor_state_vector_.push_back(dynaarm_hardware_interface_common::MotorState{ .name = joint_name });
    motor_command_vector_.push_back(dynaarm_hardware_interface_common::MotorCommand{ .name = joint_name });
  }

  RCLCPP_INFO_STREAM(logger_, "Successfully initialized dynaarm hardware interface for DynaArmHardwareInterfaceBase");

  // Check if we want to start in freeze mode (parameter is optional). If so we enable the freeze mode
  // The user than has to disable the freeze mode manually
  if (info_.hardware_parameters.find("start_in_freeze_mode") != info_.hardware_parameters.end()) {
    command_freeze_mode_ = utils::sttobool(info_.hardware_parameters.at("start_in_freeze_mode")) ? 1.0 : 0.0;
  }

  return on_init_derived(system_info.hardware_info);
}

std::vector<hardware_interface::StateInterface> DynaArmHardwareInterfaceBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto& joint_state : joint_state_vector_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_state.name, hardware_interface::HW_IF_POSITION, &joint_state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_state.name, hardware_interface::HW_IF_VELOCITY, &joint_state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_state.name, hardware_interface::HW_IF_ACCELERATION, &joint_state.acceleration));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_state.name, hardware_interface::HW_IF_EFFORT, &joint_state.effort));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_state.name, "position_commanded", &joint_state.position_commanded));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_state.name, "velocity_commanded", &joint_state.velocity_commanded));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_state.name, "acceleration_commanded",
                                                                     &joint_state.acceleration_commanded));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_state.name, "effort_commanded", &joint_state.effort_commanded));
  }

  for (auto& joint_command : joint_command_vector_) {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "p_gain_commanded", &joint_command.p_gain));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "i_gain_commanded", &joint_command.i_gain));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_command.name, "d_gain_commanded", &joint_command.d_gain));
  }

  for (auto& motor_state : motor_state_vector_) {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_state.name, "motor_position", &motor_state.position));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_state.name, "motor_velocity", &motor_state.velocity));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_state.name, "motor_acceleration", &motor_state.acceleration));
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

  for (auto& motor_command : motor_command_vector_) {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_command.name, "motor_position_commanded", &motor_command.position));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_command.name, "motor_velocity_commanded", &motor_command.velocity));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(motor_command.name, "motor_effort_commanded", &motor_command.effort));
  }

  // TODO(firesurfer) expose drive state, warnings, imu?
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynaArmHardwareInterfaceBase::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto& joint_command : joint_command_vector_) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_command.name, hardware_interface::HW_IF_POSITION, &joint_command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_command.name, hardware_interface::HW_IF_VELOCITY, &joint_command.velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_command.name, hardware_interface::HW_IF_ACCELERATION, &joint_command.acceleration));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_command.name, hardware_interface::HW_IF_EFFORT, &joint_command.effort));

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_command.name, "p_gain", &joint_command.p_gain));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_command.name, "i_gain", &joint_command.i_gain));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_command.name, "d_gain", &joint_command.d_gain));
  }
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(get_hardware_info().name, "freeze_mode", &command_freeze_mode_));

  return command_interfaces;
}

hardware_interface::CallbackReturn
DynaArmHardwareInterfaceBase::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  hardware_interface::CallbackReturn callbackReturn = on_activate_derived(previous_state);

  // Basically lock the read/write methods until on_activate has finished
  active_ = true;

  // Perform a reading once to obtain the current positions
  if (read(rclcpp::Time(), rclcpp::Duration(std::chrono::nanoseconds(0))) != hardware_interface::return_type::OK) {
    RCLCPP_ERROR_STREAM(logger_, "Could not perform initial read - this is critical");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    joint_command_vector_[i].position = joint_state_vector_[i].position;
    joint_command_vector_[i].velocity = 0.0;
    joint_command_vector_[i].acceleration = 0.0;
    joint_command_vector_[i].effort = 0.0;
    RCLCPP_INFO_STREAM(logger_, "Start position of joint: " << info_.joints[i].name
                                                            << " is: " << joint_state_vector_[i].position);
  }

  if (std::all_of(joint_command_vector_.begin(), joint_command_vector_.end(),
                  [](const auto& val) { return val.position == 0.0; })) {
    RCLCPP_FATAL_STREAM(logger_, "All initial joint readings were 0.0 - this is indicates a critical error");
    return hardware_interface::CallbackReturn::FAILURE;
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
  return hardware_interface::CallbackReturn::FAILURE;
}

hardware_interface::return_type DynaArmHardwareInterfaceBase::read(const rclcpp::Time& /*time*/,
                                                                   const rclcpp::Duration& /*period*/)
{
  if (!active_)
    return hardware_interface::return_type::OK;

  // updates the motor states. Assumed that after this function the motor_state_vector is correctly updated
  read_motor_states();

  Eigen::VectorXd motor_position(info_.joints.size());
  Eigen::VectorXd motor_velocity(info_.joints.size());
  Eigen::VectorXd motor_acceleration(info_.joints.size());
  Eigen::VectorXd motor_effort(info_.joints.size());
  Eigen::VectorXd motor_position_commanded(info_.joints.size());
  Eigen::VectorXd motor_velocity_commanded(info_.joints.size());
  Eigen::VectorXd motor_effort_commanded(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    motor_position(i) = motor_state_vector_[i].position;
    motor_velocity(i) = motor_state_vector_[i].velocity;
    motor_acceleration(i) = motor_state_vector_[i].acceleration;
    motor_effort(i) = motor_state_vector_[i].effort;
    motor_position_commanded(i) = motor_state_vector_[i].position_commanded;
    motor_velocity_commanded(i) = motor_state_vector_[i].velocity_commanded;
    motor_effort_commanded(i) = motor_state_vector_[i].effort_commanded;
  }
  // Transform the joint positions using the matrix transformation
  Eigen::VectorXd joint_position =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialCoordinates(motor_position);
  Eigen::VectorXd joint_velocity =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialCoordinates(motor_velocity);
  Eigen::VectorXd joint_acceleration =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialCoordinates(motor_acceleration);
  Eigen::VectorXd joint_effort =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialTorques(motor_effort);

  Eigen::VectorXd joint_position_commanded =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialCoordinates(motor_position_commanded);
  Eigen::VectorXd joint_velocity_commanded =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialCoordinates(motor_velocity_commanded);
  Eigen::VectorXd joint_effort_commanded =
      dynaarm_hardware_interface_common::CommandTranslator::mapFromDynaarmToSerialTorques(motor_effort_commanded);
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    joint_state_vector_[i].position = joint_position[i];
    joint_state_vector_[i].velocity = joint_velocity[i];
    joint_state_vector_[i].acceleration = joint_acceleration[i];
    joint_state_vector_[i].effort = joint_effort[i];
    joint_state_vector_[i].position_commanded = joint_position_commanded[i];
    joint_state_vector_[i].velocity_commanded = joint_velocity_commanded[i];
    // NOTE we only feedback the desired acceleration at the moment as it is solely used in the gravity compensation
    // controller
    joint_state_vector_[i].acceleration_commanded = joint_command_vector_[i].acceleration;
    joint_state_vector_[i].effort_commanded = joint_effort_commanded[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynaArmHardwareInterfaceBase::write(const rclcpp::Time& /*time*/,
                                                                    const rclcpp::Duration& /*period*/)
{
  if (!active_)
    return hardware_interface::return_type::OK;

  Eigen::VectorXd joint_position(info_.joints.size());
  Eigen::VectorXd joint_velocity(info_.joints.size());
  Eigen::VectorXd joint_effort(info_.joints.size());
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    // Clamp the outputs to the joint limits defined in the urdf
    const auto limits = info_.limits.at(info_.joints[i].name);

    if (std::isnan(joint_state_vector_[i].position_last)) {
      joint_state_vector_[i].position_last = joint_state_vector_[i].position;
    }

    if (joint_state_vector_[i].position >= limits.min_position &&
        joint_state_vector_[i].position <= limits.max_position) {
      // The if statements below make sure that the arm is not moving towards collision with itself
      // First checking if it is within limits, if yes it works under normal circumstances
      joint_position[i] = std::clamp(joint_command_vector_[i].position, limits.min_position, limits.max_position);
      joint_state_vector_[i].position_last = joint_state_vector_[i].position;
    } else if (joint_state_vector_[i].position < limits.min_position &&
               joint_command_vector_[i].position >= joint_state_vector_[i].position) {
      // If we are lower than the low_limit but the joint is moving away from collision we accept the move
      // Then it is safe to move and we Accept the new position to be commanded
      // Note that we clamp the minimum joint position value to the current position, this way we avoid jumps
      joint_position[i] =
          std::clamp(joint_command_vector_[i].position, joint_state_vector_[i].position, limits.max_position);
      joint_state_vector_[i].position_last = joint_state_vector_[i].position;
    } else if (joint_state_vector_[i].position > limits.max_position &&
               joint_command_vector_[i].position <= joint_state_vector_[i].position) {
      // Same for the upper limmit
      joint_position[i] =
          std::clamp(joint_command_vector_[i].position, limits.min_position, joint_state_vector_[i].position);
      joint_state_vector_[i].position_last = joint_state_vector_[i].position;
    } else {
      // Hold last valid position, this is why we need the position_last variable.
      joint_position[i] = joint_state_vector_[i].position_last;
    }

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

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    motor_command_vector_[i].position = motor_position[i];
    motor_command_vector_[i].velocity = motor_velocity[i];
    motor_command_vector_[i].effort = motor_effort[i];
    motor_command_vector_[i].p_gain = joint_command_vector_[i].p_gain;
    motor_command_vector_[i].i_gain = joint_command_vector_[i].i_gain;
    motor_command_vector_[i].d_gain = joint_command_vector_[i].d_gain;
  }

  // writes motor commands to the drives, simulation or directly into motor_state for mock
  write_motor_commands();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynaArmHardwareInterfaceBase::prepare_command_mode_switch(
    [[maybe_unused]] const std::vector<std::string>& start_interfaces,
    [[maybe_unused]] const std::vector<std::string>& stop_interfaces)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynaArmHardwareInterfaceBase::perform_command_mode_switch(
    [[maybe_unused]] const std::vector<std::string>& start_interfaces,
    [[maybe_unused]] const std::vector<std::string>& stop_interfaces)
{
  return hardware_interface::return_type::OK;
}

}  // namespace dynaarm_hardware_interface_base
