#pragma once

/*std*/
#include <memory>
#include <string>
#include <vector>
#include <map>

/*ROS2*/
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

/*Project*/
#include "dynaarm_status_controller_parameters.hpp"
#include "dynaarm_controllers/interface_utils.hpp"

/*msgs*/
#include <dynaarm_msgs/msg/arm_state.hpp>

namespace dynaarm_controllers {
    class StatusController: public controller_interface::ControllerInterface {
        public:
            using ArmState = dynaarm_msgs::msg::ArmState;
            using DriveState = dynaarm_msgs::msg::DriveState;


            StatusController();
            ~StatusController() = default;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    };
}