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

#include <filesystem>
#include "dynaarm_driver/dynaarm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ethercat_sdk_master/EthercatMasterSingleton.hpp"
namespace dynaarm_driver
{
hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_init_derived(const hardware_interface::HardwareInfo& system_info)
{
  // configure ethercat bus and drives
  const auto ethercat_bus = system_info.hardware_parameters.at("ethercat_bus");
  const ecat_master::EthercatMasterConfiguration ecat_master_config = {
    .name = "DynaArmHardwareInterface", .networkInterface = ethercat_bus, .timeStep = 0.001
  };  // TODO(firesurfer) set timestep according to the update rate of ros2control (or spin asynchronously)

  // Obtain an instance of the bus from the singleton - if there is no instance it will be created
  ecat_master_handle_ = ecat_master::EthercatMasterSingleton::instance().aquireMaster(ecat_master_config);

  // Every joint refers to a drive
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    const auto address = std::stoi(info_.joints[i].parameters.at("address"));
    const auto joint_name = info_.joints[i].name;

    // We do not want the joint prefix in the path for the parameter files
    auto joint_wo_prefix = joint_name;
    const auto tf_prefix = system_info.hardware_parameters.at("tf_prefix");
    // Check if the joint name starts with the tf_prefix
    if (joint_name.find(tf_prefix) == 0) {
      // Remove it from the string
      joint_wo_prefix.erase(0, tf_prefix.size());
    }

    // Obtain the parameter file for the currently processed drive
    const std::string base_directory = info_.hardware_parameters.at("drive_parameter_folder") + "/";
    std::string device_file_path = base_directory + joint_wo_prefix + ".yaml";
    // If there is no configuration available for the current joint in the passed parameter folder we load it from the
    // default folder
    if (!std::filesystem::exists(device_file_path)) {
      RCLCPP_WARN_STREAM(logger_, "No configuration found for joint: " << joint_name << " in: " << base_directory
                                                                       << " Loading drive parameters from default "
                                                                          "location");

      device_file_path =
          info_.hardware_parameters.at("drive_parameter_folder_default") + "/" + joint_wo_prefix + ".yaml";
    }
    RCLCPP_INFO_STREAM(logger_, "Drive file path " << device_file_path);

    auto drive = rsl_drive_sdk::DriveEthercatDevice::deviceFromFile(device_file_path, joint_name, address,

                                                                    rsl_drive_sdk::PdoTypeEnum::E);

    // Store in our internal list so that we can easy refer to them afterwards
    drives_.push_back(drive);

    // And attach it to the ethercat master
    if (ecat_master_handle_.ecat_master->attachDevice(drive) == false) {
      RCLCPP_ERROR_STREAM(logger_, "Could not attach the slave drive to the master.");
    }

    RCLCPP_INFO_STREAM(logger_, "Configuring drive: " << joint_name << " at bus address: " << address);
  }

  // Initialize drive init states for the pre-ControlOp validation phase
  drive_init_states_.resize(info_.joints.size());
  in_init_phase_ = true;

  RCLCPP_INFO_STREAM(logger_, "Successfully initialized dynaarm hardware interface for DynaArmHardwareInterface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // This is a bit of a work around but doesn't work otherwise
  // We separate the activation of the ethercat bus into separate stages
  // 1. Setup
  // 2. Activation - only when all handles that where given out in the setup (on_init) mark themselves as ready for
  // activation the bus will be activated
  ecat_master::EthercatMasterSingleton::instance().markAsReady(ecat_master_handle_);
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_activate_derived([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_deactivate_derived(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    auto& drive = drives_.at(i);
    drive->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 3.0, 0.01);

    rsl_drive_sdk::Command cmd;
    cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::Freeze);
    drive->setCommand(cmd);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool DynaArmHardwareInterface::read_motor_states()
{
  if (!ready_) {
    if (*ecat_master_handle_.running) {
      RCLCPP_INFO_STREAM(logger_, "Deferred initialization of arm: " << info_.name);
      ready_ = true;
      in_init_phase_ = true;  // Start pre-ControlOp initialization phase
    }
  }

  if (!ready_) {
    return false;
  }

  // ============================================================================
  // PRE-CONTROLOP INITIALIZATION PHASE
  // ============================================================================
  // Before entering ControlOp mode, we must:
  // 1. Error recovery and firmware setup
  // 2. Set Freeze mode
  // 3. Validate position stability (3 consecutive reads within 0.001 tolerance)
  // 4. Write current position as setpoint to the drive
  // 5. Transition to ControlOp
  // ============================================================================
  if (in_init_phase_) {
    bool all_init_complete = true;

    for (std::size_t i = 0; i < info_.joints.size(); i++) {

      auto& drive = drives_[i];
      auto& init_state = drive_init_states_[i];
      rsl_drive_sdk::Command cmd;

      // Phase 0: Error recovery and firmware setup (only done once per drive)
      if (!init_state.mode_set) {
        // Error recovery: Clear errors and try to recover
        rsl_drive_sdk::Statusword status_word;
        drive->getStatuswordSdo(status_word);
        if (status_word.getStateEnum() == rsl_drive_sdk::fsm::StateEnum::Error) {
          RCLCPP_WARN_STREAM(logger_, "Drive: " << info_.joints.at(i).name << " is in Error state - trying to reset");
          drive->setControlword(RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY);
          cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::Freeze);
          drive->setCommand(cmd);
          drive->updateWrite();
          drive->updateRead();
          
          

          if (!drive->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 1, 10)) {
            RCLCPP_FATAL_STREAM(logger_, "Drive: " << info_.joints[i].name << " did not go into ControlOP");
            return false;
          } else {
            RCLCPP_INFO_STREAM(logger_, "Drive: " << info_.joints.at(i).name << " went into ControlOp successfully");
          }
        }

        // Log the firmware information of the drive
        rsl_drive_sdk::common::BuildInfo info;
        drive->getBuildInfo(info);
        RCLCPP_INFO_STREAM(logger_, "Drive info: " << info_.joints[i].name << " Build date: " << info.buildDate
                                                   << " tag: " << info.gitTag << " hash: " << info.gitHash);

        rsl_drive_sdk::mode::PidGainsF gains;
        drive->getControlGains(rsl_drive_sdk::mode::ModeEnum::JointPositionVelocityTorquePidGains, gains);
        joint_command_vector_[i].p_gain = gains.getP();
        joint_command_vector_[i].i_gain = gains.getI();
        joint_command_vector_[i].d_gain = gains.getD();

        RCLCPP_INFO_STREAM(logger_, "PID Gains: " << gains);
      }

      // Get current reading
      rsl_drive_sdk::ReadingExtended reading;
      drive->getReading(reading);
      double current_position = reading.getState().getJointPosition();

      // Phase 1: Set Freeze mode (only once)
      if (!init_state.mode_set) {        
        init_state.current_mode = rsl_drive_sdk::mode::ModeEnum::Freeze;
        cmd.setModeEnum(init_state.current_mode);
        drive->setCommand(cmd);
        init_state.mode_set = true;
        RCLCPP_INFO_STREAM(logger_, "Drive " << i << ": Mode set to Freeze");
      }

      // Phase 2: Validate position stability
      if (!init_state.position_validated) {
        // Check if position is not 0.0 (or very close to it across all joints)
        if (std::abs(current_position) < 1e-6) {
          RCLCPP_WARN_STREAM(logger_, "Drive " << i << ": Position reading is 0.0 - retrying position validation");
          init_state.position_readings.clear();
          init_state.stable_read_count = 0;
          init_state.position_readings.push_back(current_position);
        } else {
          // Add position to readings
          init_state.position_readings.push_back(current_position);

          // Keep only the last REQUIRED_STABLE_READS readings
          if (init_state.position_readings.size() > REQUIRED_STABLE_READS) {
            init_state.position_readings.erase(init_state.position_readings.begin());
          }

          // Check if we have enough readings
          if (init_state.position_readings.size() == REQUIRED_STABLE_READS) {
            // Check if all readings are within tolerance
            bool all_stable = true;
            double first_position = init_state.position_readings[0];

            for (std::size_t j = 1; j < init_state.position_readings.size(); j++) {
              if (std::abs(init_state.position_readings[j] - first_position) > POSITION_TOLERANCE) {
                all_stable = false;
                RCLCPP_WARN_STREAM(logger_, "Drive " << i << ": Position not stable - reading " << j + 1
                                                     << " (" << init_state.position_readings[j]
                                                     << ") differs from first reading (" << first_position
                                                     << ") by " << std::abs(init_state.position_readings[j] - first_position)
                                                     << " - retrying...");
                break;
              }
            }

            if (all_stable) {
              init_state.position_validated = true;
              RCLCPP_INFO_STREAM(logger_, "Drive " << i << ": Position validated at " << first_position);
            } else {
              // Reset for retry
              init_state.position_readings.clear();
            }
          }
        }
      }

      // Phase 3: Write position as setpoint (after position is validated)
      if (init_state.position_validated && !init_state.controlop_transition_initiated) {
        rsl_drive_sdk::Command cmd;
        double setpoint_position = init_state.position_readings[0];
        cmd.setJointPosition(setpoint_position);
        cmd.setModeEnum(init_state.current_mode);
        drive->setCommand(cmd);
        init_state.controlop_transition_initiated = true;
        RCLCPP_INFO_STREAM(logger_, "Drive " << i << ": Position " << setpoint_position
                                             << " written as setpoint to drive");
      }

      // Check if all conditions are met for this drive
      if (!init_state.position_validated || !init_state.mode_set || !init_state.controlop_transition_initiated) {
        all_init_complete = false;
      }
    }

    // If all drives completed initialization, transition to ControlOp mode
    if (all_init_complete) {
      RCLCPP_INFO_STREAM(logger_, "All drives ready - transitioning to ControlOp mode");
      for (std::size_t i = 0; i < drives_.size(); i++) {
        if (!drives_[i]->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 1, 10)) {
          RCLCPP_FATAL_STREAM(logger_, "Drive " << i << " failed to transition to ControlOp");
          return false;
        }
        RCLCPP_INFO_STREAM(logger_, "Drive " << i << " successfully transitioned to ControlOp");
      }
      in_init_phase_ = false;
    } else {
      // Still in init phase - read and update state but don't process further
      for (std::size_t i = 0; i < info_.joints.size(); i++) {
        rsl_drive_sdk::ReadingExtended reading;
        drives_[i]->getReading(reading);
        auto state = reading.getState();

        motor_state_vector_[i].position = state.getJointPosition();
        motor_state_vector_[i].velocity = state.getJointVelocity();
        motor_state_vector_[i].acceleration = state.getJointAcceleration();
        motor_state_vector_[i].effort = state.getJointTorque();

        motor_state_vector_[i].position_commanded = reading.getCommanded().getJointPosition();
        motor_state_vector_[i].velocity_commanded = reading.getCommanded().getJointVelocity();
        motor_state_vector_[i].effort_commanded = reading.getCommanded().getJointTorque();

        motor_state_vector_[i].temperature = state.getTemperature();
        motor_state_vector_[i].temperature_coil_A = state.getCoilTemp1();
        motor_state_vector_[i].temperature_coil_B = state.getCoilTemp2();
        motor_state_vector_[i].temperature_coil_C = state.getCoilTemp3();
        motor_state_vector_[i].bus_voltage = state.getVoltage();
      }
      return true;
    }
  }

  // Normal operation (after init phase)
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    // Get a reading from the specific drive and
    rsl_drive_sdk::ReadingExtended reading;
    // NOTE: getReading uses a recursive mutex -> It would be better if we could do something like: tryLock and if we
    // can't look then we try again in the next cycle
    drives_[i]->getReading(reading);  // Use [ ] instead of at for performance reasons

    // And update the state vector so that controllers can read the current state
    auto state = reading.getState();

    motor_state_vector_[i].position = state.getJointPosition();
    motor_state_vector_[i].velocity = state.getJointVelocity();
    motor_state_vector_[i].acceleration = state.getJointAcceleration();
    motor_state_vector_[i].effort = state.getJointTorque();

    motor_state_vector_[i].position_commanded = reading.getCommanded().getJointPosition();
    motor_state_vector_[i].velocity_commanded = reading.getCommanded().getJointVelocity();
    motor_state_vector_[i].effort_commanded = reading.getCommanded().getJointTorque();

    motor_state_vector_[i].temperature = state.getTemperature();
    motor_state_vector_[i].temperature_coil_A = state.getCoilTemp1();
    motor_state_vector_[i].temperature_coil_B = state.getCoilTemp2();
    motor_state_vector_[i].temperature_coil_C = state.getCoilTemp3();
    motor_state_vector_[i].bus_voltage = state.getVoltage();
  }

  return true;
}

void DynaArmHardwareInterface::write_motor_commands()
{
  if (!ready_) {
    return;
  }

  // Do not write commands during initialization phase
  if (in_init_phase_) {
    RCLCPP_DEBUG_STREAM(logger_, "Skipping motor commands during initialization phase");
    return;
  }

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    // Obtain reference to the specific drive
    auto& drive = drives_[i];

    // Only write the command if we are already in the correct state
    if (drive->goalStateHasBeenReached()) {
      // Convert command vector into an rsl_drive_sdk::Command
      // Make sure to be in the right mode
      rsl_drive_sdk::Command cmd;

      rsl_drive_sdk::mode::PidGainsF gains;
      gains.setP(motor_command_vector_[i].p_gain);
      gains.setI(motor_command_vector_[i].i_gain);
      gains.setD(motor_command_vector_[i].d_gain);

      cmd.setJointPosition(motor_command_vector_[i].position);
      cmd.setJointVelocity(motor_command_vector_[i].velocity);
      cmd.setJointTorque(motor_command_vector_[i].effort);
      cmd.setPidGains(gains);

      if (command_freeze_mode_ == 1.0) {
        cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::Freeze);
      } else {
        cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::JointPositionVelocityTorquePidGains);
      }

      // We always fill all command fields but depending on the mode only a subset is used
      drive->setCommand(cmd);
    }
  }
}

DynaArmHardwareInterface::~DynaArmHardwareInterface()
{
  RCLCPP_INFO_STREAM(logger_, "Destructor of DynaArm Hardware Interface called");
  ecat_master::EthercatMasterSingleton::instance().releaseMaster(ecat_master_handle_);
  ecat_master_handle_.ecat_master.reset();
}
}  // namespace dynaarm_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_driver::DynaArmHardwareInterface, hardware_interface::SystemInterface)
