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

#pragma once

/* ROS 2 */
#include <controller_interface/controller_interface.hpp>

/* msgs */
#include <sensor_msgs/msg/joy.hpp>

/* project */
#include <dynaarm_controllers/dynaarm_safety_controller_parameters.hpp>
#include <dynaarm_controllers/interface_utils.hpp>

namespace dynaarm_controllers
{
class SafetyController : public controller_interface::ControllerInterface
{
public:
  using JoySubscription = rclcpp::Subscription<sensor_msgs::msg::Joy>;

  SafetyController();
  ~SafetyController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  bool gamepad_connected_ = false;
  bool emergency_stop_active_ = true;
  bool button_released_ = true;

  int emergency_stop_button_ = 8;
  rclcpp::Time emergency_stop_pressed_time_;

  void check_gamepad_connection();
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  std::vector<JoySubscription::SharedPtr> joy_subscriptions_;
  CommandInterfaceReferences emergency_stop_interface_;

  // Access to controller parameters via generate_parameter_library
  std::unique_ptr<dynaarm_safety_controller::ParamListener> param_listener_;
  dynaarm_safety_controller::Params params_;
};

}  // namespace dynaarm_controllers
