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

 
/* std */
#include <std_msgs/msg/bool.hpp>

#include <dynaarm_controllers/dynaarm_safety_controller.hpp>
#include <controller_interface/helpers.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace dynaarm_controllers
{
SafetyController::SafetyController() 
{  
}

controller_interface::InterfaceConfiguration SafetyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(params_.arm_name + "/emergency_stop");  
  return config;
}

controller_interface::InterfaceConfiguration SafetyController::state_interface_configuration() const
{
    // Claim the necessary state interfaces
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    return config;
}

controller_interface::CallbackReturn SafetyController::on_init()
{
  try 
  {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<dynaarm_safety_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } 
  catch (const std::exception& e) 
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyController::on_configure(const rclcpp_lifecycle::State&)
{
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  joy_subscriptions_.push_back(get_node()->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&SafetyController::joy_callback, this, std::placeholders::_1)));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyController::on_activate(const rclcpp_lifecycle::State&)
{
  emergency_stop_interface_.clear();

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, std::vector<std::string>{ params_.arm_name }, "emergency_stop", emergency_stop_interface_)) 
  {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interface - emergency_stop");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Read the initial value from the hardware interface
  emergency_stop_active_ = static_cast<bool>(emergency_stop_interface_.at(0).get().get_value());
  RCLCPP_WARN(get_node()->get_logger(), "Emergency Stop initial state: %s", emergency_stop_active_ ? "ACTIVE" : "INACTIVE");

  check_gamepad_connection();

  if (!gamepad_connected_) {
    RCLCPP_ERROR(get_node()->get_logger(), "No gamepad detected. Please connect a gamepad and relaunch.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyController::on_deactivate(const rclcpp_lifecycle::State&)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SafetyController::update(const rclcpp::Time&, const rclcpp::Duration&)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  // Set the emergency stop value in the hardware interface
  (void)emergency_stop_interface_.at(0).get().set_value(static_cast<double>(emergency_stop_active_));  

  return controller_interface::return_type::OK;
}

void SafetyController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!gamepad_connected_)
  {
    gamepad_connected_ = true;        
  }

  if (emergency_stop_button_ < 0 || emergency_stop_button_ >= static_cast<int>(msg->buttons.size()))
  {
    return;  // Ignore invalid button index
  }

  bool pressed = msg->buttons[emergency_stop_button_];

  if (emergency_stop_active_)  
  {
    // If emergency stop is active, the button must be held for 3 seconds to disable it
    if (pressed)
    {
      if (emergency_stop_pressed_time_.nanoseconds() == 0)  
      {
        emergency_stop_pressed_time_ = get_node()->now();
      }

      if ((get_node()->now() - emergency_stop_pressed_time_).seconds() >= 3)
      {
        RCLCPP_INFO(get_node()->get_logger(), "Emergency stop released. Resuming control.");
        emergency_stop_active_ = false;
        button_released_ = false;  // Require button release before reactivating
        emergency_stop_pressed_time_ = rclcpp::Time(0); // Reset timer
      }
    }
    else
    {
      // Button released, reset the timer
      emergency_stop_pressed_time_ = rclcpp::Time(0);
    }
  }
  else  
  {
    // Wait for button release before allowing activation again
    if (!pressed)
    {
      button_released_ = true;
    }

    // Reactivate emergency stop only if button was released first
    if (pressed && button_released_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "EMERGENCY STOP ACTIVATED!");
      emergency_stop_active_ = true;
    }
  }
}

void SafetyController::check_gamepad_connection()
{
    rclcpp::Rate rate(1.0);
    int attempts = 0;
    while (rclcpp::ok() && !gamepad_connected_)
    {
        RCLCPP_WARN(get_node()->get_logger(), "Waiting for gamepad connection... (Attempt %d)", attempts);
        attempts++;

        if (attempts > 20) 
        {            
            return;
        }

        rate.sleep();
    }
    RCLCPP_INFO(get_node()->get_logger(), "Gamepad detected! Proceeding...");
}
} // namespace dynaarm_controllers

PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::SafetyController, controller_interface::ControllerInterface)
