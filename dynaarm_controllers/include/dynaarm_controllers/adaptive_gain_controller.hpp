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

/* std */
#include <vector>
#include <string>
#include <memory>

/* pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

/* ros2_control */
#include <controller_interface/controller_interface.hpp>

/* project */
#include <dynaarm_controllers/adaptive_gain_controller_parameters.hpp>
#include <dynaarm_controllers/interface_utils.hpp>

namespace dynaarm_controllers
{

class AdaptiveGainController : public controller_interface::ControllerInterface
{
public:
  AdaptiveGainController();
  ~AdaptiveGainController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;  

private:
  std::unique_ptr<adaptive_gain_controller::ParamListener> param_listener_;
  adaptive_gain_controller::Params params_;

  std::vector<double> base_p_gains_;
  std::vector<double> base_i_gains_;
  std::vector<double> base_d_gains_;

  CommandInterfaceReferences joint_p_gain_command_interfaces_;
  CommandInterfaceReferences joint_i_gain_command_interfaces_;
  CommandInterfaceReferences joint_d_gain_command_interfaces_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interfaces_;

  pinocchio::Model pinocchio_model_;
  pinocchio::Data pinocchio_data_;

  std::atomic_bool active_{ false };
};

}  // namespace dynaarm_controllers
