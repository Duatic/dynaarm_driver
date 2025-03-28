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
  ecat_master_ = std::make_shared<ecat_master::EthercatMaster>();
  ecat_master_->loadEthercatMasterConfiguration(ecat_master_config);

  // Every joint refers to a drive
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    const auto address = std::stoi(info_.joints[i].parameters.at("address"));
    const auto joint_name = info_.joints[i].name;

    // Obtain the parameter file for the currently processed drive
    const std::string base_directory = info_.hardware_parameters.at("drive_parameter_folder") + "/";
    std::string device_file_path = base_directory + joint_name + ".yaml";
    // If there is no configuration available for the current joint in the passed parameter folder we load it from the
    // default folder
    if (!std::filesystem::exists(device_file_path)) {
      RCLCPP_WARN_STREAM(logger_, "No configuration found for joint: " << joint_name << " in: " << base_directory
                                                                       << " Loading drive parameters from default "
                                                                          "location");
      device_file_path = info_.hardware_parameters.at("drive_parameter_folder_default") + "/" + joint_name + ".yaml";
    }
    RCLCPP_INFO_STREAM(logger_, "Drive file path " << device_file_path);

    auto drive = rsl_drive_sdk::DriveEthercatDevice::deviceFromFile(device_file_path, joint_name, address,

                                                                    rsl_drive_sdk::PdoTypeEnum::E);

    // Store in our internal list so that we can easy refer to them afterwards
    drives_.push_back(drive);

    // And attach it to the ethercat master
    if (ecat_master_->attachDevice(drive) == false) {
      RCLCPP_ERROR_STREAM(logger_, "Could not attach the slave drive to the master.");
    }

    RCLCPP_INFO_STREAM(logger_, "Configuring drive: " << joint_name << " at bus address: " << address);
  }

  if (ecat_master_->startup(startupAbortFlag_) == false) {
    RCLCPP_ERROR_STREAM(logger_, "Could not start the Ethercat Master.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(
      logger_, "Successfully started Ethercat Master on Network Interface: " << ecat_master_->getBusPtr()->getName());

  // Actually run the ethercat master in a separate thread
  ecat_worker_thread_ = std::make_unique<std::thread>([this] {
    // A rt priority > 48 seems to starve other processes on some systems.
    if (ecat_master_->setRealtimePriority(48) == false) {
      RCLCPP_WARN_STREAM(logger_, "Could not increase thread priority - check user privileges.");
    }

    if (ecat_master_->activate()) {
      RCLCPP_INFO_STREAM(logger_, "Activated the Bus: " << ecat_master_->getBusPtr()->getName());
    }

    while (!abrtFlag_) {
      ecat_master_->update(ecat_master::UpdateMode::StandaloneEnforceStep);
    }
    ecat_master_->deactivate();
  });

  RCLCPP_INFO_STREAM(logger_, "Successfully initialized dynaarm hardware interface for DynaArmHardwareInterface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_activate_derived([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    auto& drive = drives_[i];
    // In case we are in error state clear the error and try again
    rsl_drive_sdk::Statusword status_word;
    drive->getStatuswordSdo(status_word);
    if (status_word.getStateEnum() == rsl_drive_sdk::fsm::StateEnum::Error) {
      RCLCPP_WARN_STREAM(logger_, "Drive: " << info_.joints.at(i).name << " is in Error state - trying to reset");
      drive->setControlword(RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY);
      drive->updateWrite();
      drive->updateRead();
      if (!drive->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 1, 10)) {
        RCLCPP_FATAL_STREAM(logger_, "Drive: " << info_.joints[i].name << " did not go into ControlOP");
      } else {
        RCLCPP_INFO_STREAM(logger_, "Drive: " << info_.joints.at(i).name << " went into ControlOp successfully");
      }
    }
  }

  // On activate is already in the realtime loop (on_configure would be in the non_rt loop)
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    auto& drive = drives_[i];

    // Put into controlOP, in blocking mode.
    if (!drive->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 1, 10)) {
      RCLCPP_FATAL_STREAM(logger_, "Drive: " << info_.joints[i].name
                                             << " did not go into ControlOP - this is trouble some and a reason to "
                                                "abort. Try to reboot the hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Log the firmware information of the drive. Might be useful for debugging issues at customer
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

void DynaArmHardwareInterface::read_motor_states()
{
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
}

void DynaArmHardwareInterface::write_motor_commands()
{
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    // Obtain reference to the specific drive
    auto& drive = drives_[i];

    // Only write the command if we are already in the correct state
    if (drive->goalStateHasBeenReached()) {
      // Convert command vector into an rsl_drive_sdk::Command
      // Make sure to be in the right mode
      rsl_drive_sdk::Command cmd;

      rsl_drive_sdk::mode::PidGainsF gains;
      const float p_gain = motor_command_vector_[i].p_gain;
      const float i_gain = motor_command_vector_[i].i_gain;
      const float d_gain = motor_command_vector_[i].d_gain;
      gains.setP(p_gain);
      gains.setI(i_gain);
      gains.setD(d_gain);

      cmd.setJointPosition(motor_command_vector_[i].position);
      cmd.setJointVelocity(motor_command_vector_[i].velocity);
      cmd.setJointTorque(motor_command_vector_[i].effort);
      cmd.setPidGains(gains);

      if (command_freeze_mode_ == 1.0) {
        cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::Freeze);
      } else if (p == 0.0f && i == 0.0f && d == 0.0f) {
        cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::JointTorque);
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
  // call preShutdown before terminating the cyclic PDO communication!!
  if (ecat_master_) {
    ecat_master_->preShutdown(true);
  }
  RCLCPP_INFO_STREAM(logger_, "PreShutdown ethercat master and all slaves.");

  // tell the ecat master thread to end and join it if possible
  abrtFlag_ = true;
  if (ecat_worker_thread_) {
    if (ecat_worker_thread_->joinable()) {
      ecat_worker_thread_->join();
    }
  }

  ecat_master_->shutdown();

  RCLCPP_INFO_STREAM(logger_, "Fully shutdown.");
}
}  // namespace dynaarm_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_driver::DynaArmHardwareInterface, hardware_interface::SystemInterface)
