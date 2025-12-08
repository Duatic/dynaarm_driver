// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dynaarm_gz_ros2_control/dynaarm_system.hpp"

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <filesystem>
#include <gz/physics/Geometry.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <yaml-cpp/yaml.h>
struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's type.
  sdf::JointType joint_type;

  /// \brief Joint's axis.
  sdf::JointAxis joint_axis;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief Current cmd joint force
  double joint_torque_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  double p_gain = 0.0;  // Proportional gain for position control
  double d_gain = 0.0;  // Derivative gain for position control
  double i_gain = 0.0;  // Integral gain for position control

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;
};

class dynaarm_gz_ros2_control::GazeboSimSystemPrivate
{
public:
  GazeboSimSystemPrivate() = default;
  ~GazeboSimSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager* ecm;

  /// \brief controller update rate
  unsigned int update_rate;

  /// \brief Gazebo communication node.
  GZ_TRANSPORT_NAMESPACE Node node;
};

namespace dynaarm_gz_ros2_control
{
bool GazeboSimSystem::initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, sim::Entity>& enableJoints,
                              const hardware_interface::HardwareInfo& hardware_info, sim::EntityComponentManager& _ecm,
                              unsigned int update_rate)
{
  this->dataPtr = std::make_unique<GazeboSimSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->n_dof_ = hardware_info.joints.size();
  this->dataPtr->update_rate = update_rate;

  RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    auto& joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it_joint = enableJoints.find(joint_name);

    if (joint_name == this->el_fle_joint_name) {
      this->el_fle_idx = j;
    } else if (joint_name == this->sh_fle_joint_name) {
      this->sh_fle_idx = j;
    }

    if (it_joint == enableJoints.end()) {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(),
                         "Skipping joint in the URDF named '" << joint_name << "' which is not in the gazebo model.");
      continue;
    }

    const std::string drive_mode = hardware_info.hardware_parameters.at("drive_mode");

    const std::string base_directory = hardware_info.hardware_parameters.at("drive_parameter_folder") + "/";
    std::string device_file_path = base_directory + joint_name + ".yaml";

    if (!std::filesystem::exists(device_file_path)) {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "No configuration found for joint: " << joint_name
                                                                                       << " in: " << base_directory
                                                                                       << " Loading drive parameters "
                                                                                          "from default "
                                                                                          "location");

      device_file_path = info_.hardware_parameters.at("drive_parameter_folder_default") + "/" + joint_name + ".yaml";
    }
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Drive file path " << device_file_path);
    YAML::Node config;
    try {
      config = YAML::LoadFile(device_file_path);
    } catch (const YAML::BadFile& e) {
      RCLCPP_ERROR_STREAM(this->nh_->get_logger(),
                          "Failed to load drive parameters from file: " << device_file_path << ", error: " << e.what());
      return false;
    } catch (const YAML::ParserException& e) {
      RCLCPP_ERROR_STREAM(this->nh_->get_logger(),
                          "YAML parser error in file: " << device_file_path << ", error: " << e.what());
      return false;
    }

    bool found_drive_mode = false;
    for (const auto& drive_parameters : config["modes"]) {
      if (drive_parameters["name"].as<std::string>() == drive_mode) {
        this->dataPtr->joints_[j].p_gain = drive_parameters["gains"]["p"].as<double>();
        this->dataPtr->joints_[j].d_gain = drive_parameters["gains"]["d"].as<double>();
        this->dataPtr->joints_[j].i_gain = drive_parameters["gains"]["i"].as<double>();
        found_drive_mode = true;
        break;
      }
    }
    if (!found_drive_mode) {
      RCLCPP_ERROR_STREAM(this->nh_->get_logger(),
                          "Drive mode '" << drive_mode << "' not found in file: " << device_file_path);
      return false;
    }

    // Load the PID Drive parameters
    RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                       "THE PID gains for joint: " << joint_name << " have been set to: "
                                                   << "P: " << this->dataPtr->joints_[j].p_gain
                                                   << ", D: " << this->dataPtr->joints_[j].d_gain
                                                   << ", I: " << this->dataPtr->joints_[j].i_gain);

    sim::Entity simjoint = enableJoints[joint_name];
    this->dataPtr->joints_[j].sim_joint = simjoint;
    this->dataPtr->joints_[j].joint_type = _ecm.Component<sim::components::JointType>(simjoint)->Data();
    this->dataPtr->joints_[j].joint_axis = _ecm.Component<sim::components::JointAxis>(simjoint)->Data();

    if (!_ecm.EntityHasComponentType(simjoint, sim::components::JointPosition().TypeId())) {
      _ecm.CreateComponent(simjoint, sim::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(simjoint, sim::components::JointVelocity().TypeId())) {
      _ecm.CreateComponent(simjoint, sim::components::JointVelocity());
    }

    // Create joint transmitted wrench component if one doesn't exist
    if (!_ecm.EntityHasComponentType(simjoint, sim::components::JointTransmittedWrench().TypeId())) {
      _ecm.CreateComponent(simjoint, sim::components::JointTransmittedWrench());
    }

    // Accept this joint and continue configuration

    auto get_initial_value = [this, joint_name](const hardware_interface::InterfaceInfo& interface_info) {
      double initial_value{ 0.0 };
      if (!interface_info.initial_value.empty()) {
        try {
          initial_value = hardware_interface::stod(interface_info.initial_value);
        } catch (std::invalid_argument&) {
          RCLCPP_ERROR_STREAM(this->nh_->get_logger(),
                              "Failed converting initial_value string to real number for the joint "
                                  << joint_name << " and state interface " << interface_info.name
                                  << ". Actual value of parameter: " << interface_info.initial_value
                                  << ". Initial value will be set to 0.0");
          throw std::invalid_argument("Failed converting initial_value string");
        }
      }
      return initial_value;
    };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION,
                                                      &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY,
                                                      &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT,
                                                      &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
    }

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "position") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION,
                                                        &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY,
                                                        &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      } else if (joint_info.command_interfaces[i].name == "effort") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT,
                                                        &this->dataPtr->joints_[j].joint_effort_cmd);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      } else if (joint_info.command_interfaces[i].name == "p_gain") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, "p_gain", &this->dataPtr->joints_[j].p_gain);
      } else if (joint_info.command_interfaces[i].name == "d_gain") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, "d_gain", &this->dataPtr->joints_[j].d_gain);
      } else if (joint_info.command_interfaces[i].name == "i_gain") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, "i_gain", &this->dataPtr->joints_[j].i_gain);
      }

      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->joints_[j].joint_position = initial_position;
        this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[j].sim_joint,
                                            sim::components::JointPositionReset({ initial_position }));
      }
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
        this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[j].sim_joint,
                                            sim::components::JointVelocityReset({ initial_velocity }));
      }
    }

    // check if joint is actuated (has command interfaces) or passive
    this->dataPtr->joints_[j].is_actuated = (joint_info.command_interfaces.size() > 0);
  }

  if (this->el_fle_idx == -1 || this->sh_fle_idx == -1) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "Elbow flexion joint or shoulder flexion joint not found in the "
                                                 "hardware info. "
                                                 "Please check the URDF and hardware info configuration.");
    // Print all available joint names
    for (const auto& joint : hardware_info.joints) {
      RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "Available joint: " << joint.name);
    }
    return false;
  }
  // registerSensors(hardware_info);

  return true;
}

CallbackReturn GazeboSimSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSimSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GazeboSimSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> GazeboSimSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn GazeboSimSystem::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn GazeboSimSystem::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type GazeboSimSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    // Get the joint velocity
    const auto* jointVelocity =
        this->dataPtr->ecm->Component<sim::components::JointVelocity>(this->dataPtr->joints_[i].sim_joint);

    // Get the joint force via joint transmitted wrench
    const auto* jointWrench =
        this->dataPtr->ecm->Component<sim::components::JointTransmittedWrench>(this->dataPtr->joints_[i].sim_joint);

    // Get the joint position
    const auto* jointPositions =
        this->dataPtr->ecm->Component<sim::components::JointPosition>(this->dataPtr->joints_[i].sim_joint);

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    gz::physics::Vector3d force_or_torque;
    if (this->dataPtr->joints_[i].joint_type == sdf::JointType::PRISMATIC) {
      force_or_torque = { jointWrench->Data().force().x(), jointWrench->Data().force().y(),
                          jointWrench->Data().force().z() };
    } else {  // REVOLUTE and CONTINUOUS
      force_or_torque = { jointWrench->Data().torque().x(), jointWrench->Data().torque().y(),
                          jointWrench->Data().torque().z() };
    }
    // Calculate the scalar effort along the joint axis
    this->dataPtr->joints_[i].joint_effort = force_or_torque.dot(gz::physics::Vector3d{
        this->dataPtr->joints_[i].joint_axis.Xyz()[0], this->dataPtr->joints_[i].joint_axis.Xyz()[1],
        this->dataPtr->joints_[i].joint_axis.Xyz()[2] });
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSimSystem::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/, const std::vector<std::string>& /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSimSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    double p_gain = this->dataPtr->joints_[i].p_gain;
    double d_gain = this->dataPtr->joints_[i].d_gain;
    double i_gain = this->dataPtr->joints_[i].i_gain;

    if (i_gain != 0.0) {
      // TODO(zrene): Implement i_gain support.
      RCLCPP_WARN_THROTTLE(this->nh_->get_logger(), *this->nh_->get_clock(), 1000,
                           "i_gain is not supported in GazeboSimSystem, please set it to 0.0 for joint: %s",
                           this->dataPtr->joints_[i].name.c_str());
    }

    double p_error = this->dataPtr->joints_[i].joint_position_cmd - this->dataPtr->joints_[i].joint_position;
    double d_error = this->dataPtr->joints_[i].joint_velocity_cmd - this->dataPtr->joints_[i].joint_velocity;
    double des_torque = this->dataPtr->joints_[i].joint_effort_cmd;

    double desired_drive_torque = p_error * p_gain + d_error * d_gain + des_torque * i_gain;
    this->dataPtr->joints_[i].joint_torque_cmd = desired_drive_torque;
  }

  // Convert from serial configuration to parallel configuration
  this->dataPtr->joints_[this->sh_fle_idx].joint_torque_cmd -=
      this->dataPtr->joints_[this->el_fle_idx].joint_torque_cmd;

  // TODO(zrene): Implement actuator model for actual torque
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    this->dataPtr->joints_[i].joint_torque_cmd =
        std::clamp(this->dataPtr->joints_[i].joint_torque_cmd, -this->dataPtr->joints_[i].joint_axis.Effort(),
                   this->dataPtr->joints_[i].joint_axis.Effort());
  }

  // Convert back to serial configuration
  this->dataPtr->joints_[this->sh_fle_idx].joint_torque_cmd +=
      this->dataPtr->joints_[this->el_fle_idx].joint_torque_cmd;

  // Write to gazebo
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    auto force = this->dataPtr->ecm->Component<sim::components::JointForceCmd>(this->dataPtr->joints_[i].sim_joint);
    if (force == nullptr) {
      this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointForceCmd({ this->dataPtr->joints_[i].joint_torque_cmd }));
    } else {
      force->Data()[0] += this->dataPtr->joints_[i].joint_torque_cmd;
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace dynaarm_gz_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(dynaarm_gz_ros2_control::GazeboSimSystem, gz_ros2_control::GazeboSimSystemInterface)
// for backward compatibility with Ignition Gazebo
PLUGINLIB_EXPORT_CLASS(ign_ros2_control::IgnitionSystem, gz_ros2_control::GazeboSimSystemInterface)
