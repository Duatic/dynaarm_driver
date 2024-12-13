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

/*std*/
#include <memory>
#include <string>
#include <vector>
#include <map>

/*ROS2*/
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/logging.hpp>
/*Project*/
#include "dynaarm_status_controller_parameters.hpp"
#include "dynaarm_controllers/interface_utils.hpp"

/*msgs*/
#include <dynaarm_msgs/msg/arm_state.hpp>

namespace dynaarm_controllers
{
class StatusController : public controller_interface::ControllerInterface
{
public:
  // Some convenient typedef for easier handling of messages
  using ArmState = dynaarm_msgs::msg::ArmState;
  using ArmStatePublisher = realtime_tools::RealtimePublisher<ArmState>;
  using DriveState = dynaarm_msgs::msg::DriveState;

  StatusController();
  ~StatusController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Access to controller parameters via generate_parameter_library
  std::unique_ptr<dynaarm_status_controller::ParamListener> param_listener_;
  dynaarm_status_controller::Params params_;

  // The actual state publisher and it's realtime wrapper
  rclcpp::Publisher<ArmState>::SharedPtr arm_state_pub_;
  std::unique_ptr<ArmStatePublisher> arm_state_pub_rt_;

  // State interface references
  StateInterfaceReferences joint_position_interfaces_;
  StateInterfaceReferences joint_velocity_interfaces_;
  StateInterfaceReferences joint_effort_interfaces_;
  StateInterfaceReferences joint_position_commanded_interfaces_;
  StateInterfaceReferences joint_velocity_commanded_interfaces_;
  StateInterfaceReferences joint_effort_commanded_interfaces_;
  StateInterfaceReferences joint_temperature_system_interfaces_;
  StateInterfaceReferences joint_temperature_phase_a_interfaces_;
  StateInterfaceReferences joint_temperature_phase_b_interfaces_;
  StateInterfaceReferences joint_temperature_phase_c_interfaces_;
  StateInterfaceReferences joint_bus_voltage_interfaces_;
};
}  // namespace dynaarm_controllers
