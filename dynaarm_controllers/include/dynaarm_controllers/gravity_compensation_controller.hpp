#ifndef DYNAARM_GRAVITY_COMPENSATION_CONTROLLERS_HPP_
#define DYNAARM_GRAVITY_COMPENSATION_CONTROLLERS_HPP_

#include "controller_interface/controller_interface.hpp"
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "realtime_tools/realtime_buffer.h"

// Pinocchio
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/sample-models.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/rnea.hpp>

// ROS 2
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace dynaarm_controllers
{
class GravityCompensationController : public controller_interface::ControllerInterface
{
    public:        
        GravityCompensationController();        
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    
    protected:
    
        pinocchio::Model model;
        pinocchio::Data data;
        
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;
               
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_effort_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_p_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_i_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_d_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_effort_state_interface_;

        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *> command_interface_map_ = {
            {"position", &joint_position_command_interface_},
            {"velocity", &joint_velocity_command_interface_},
            {"effort", &joint_effort_command_interface_},
            {"p", &joint_p_command_interface_},
            {"i", &joint_i_command_interface_},
            {"d", &joint_d_command_interface_}};

        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *> state_interface_map_ = {
            {"position", &joint_position_state_interface_},
            {"velocity", &joint_velocity_state_interface_},
            {"effort", &joint_effort_state_interface_}};
};
}


#endif  // DYNAARM_GRAVITY_COMPENSATION_CONTROLLERS_HPP_