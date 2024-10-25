#include "dynaarm_controllers/dynaarm_status_controller.hpp"


namespace dynaarm_controllers {
            StatusController::StatusController() {}
            controller_interface::InterfaceConfiguration StatusController::command_interface_configuration()const{}

            controller_interface::InterfaceConfiguration StatusController::state_interface_configuration()const {}

            controller_interface::CallbackReturn StatusController::on_init() {}

            controller_interface::CallbackReturn StatusController::on_configure(const rclcpp_lifecycle::State &previous_state) {}
            controller_interface::CallbackReturn StatusController::on_activate(const rclcpp_lifecycle::State &previous_state) {}

            controller_interface::CallbackReturn StatusController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {}

            controller_interface::return_type StatusController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {}
}