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

#ifndef dynaarm_gz_ros2_control__dynaarm_system_HPP_
#define dynaarm_gz_ros2_control__dynaarm_system_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace dynaarm_gz_ros2_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declaration
class GazeboSimSystemPrivate;

class GazeboSimSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  // Documentation Inherited
  CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  // Documentation Inherited
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Documentation Inherited
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Documentation Inherited
  bool initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, sim::Entity>& joints,
               const hardware_interface::HardwareInfo& hardware_info, sim::EntityComponentManager& _ecm,
               unsigned int update_rate) override;

private:
  /// \brief Private data class
  std::unique_ptr<GazeboSimSystemPrivate> dataPtr;

  /// \brief Index of the elbow flexion joint
  unsigned int el_fle_idx = -1;
  /// \brief Name of the elbow flexion joint
  std::string el_fle_joint_name = "elbow_flexion";
  /// \brief Index of the shoulder flexion joint
  unsigned int sh_fle_idx = -1;
  /// \brief Name of the shoulder flexion joint
  std::string sh_fle_joint_name = "shoulder_flexion";
};

}  // namespace dynaarm_gz_ros2_control

// for backward compatibility
namespace ign_ros2_control
{
using IgnitionSystem = dynaarm_gz_ros2_control::GazeboSimSystem;
}  // namespace ign_ros2_control

#endif  // dynaarm_gz_ros2_control__dynaarm_system_HPP_
