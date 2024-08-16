#ifndef DYNAARM_POSITION_CONTROLLERS_HPP_
#define DYNAARM_POSITION_CONTROLLERS_HPP_

#include "controller_interface/controller_interface.hpp"
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "realtime_tools/realtime_buffer.h"
#include <rclcpp/clock.hpp>
#include <rclcpp/time_source.hpp>

namespace dynaarm_controllers
{
    class PositionController : public controller_interface::ControllerInterface
    {
    public:
        CONTROLLER_INTERFACE_PUBLIC
        PositionController();
        void read_state_from_hardware();
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

    private:
        double interpolateNextStep(double current_position, double target_position, double &step_size,
                                   double max_velocity,
                                   double delta_time);

    protected:
        rclcpp::Clock clock_;
        std::chrono::nanoseconds previous_time_;
        std::vector<double> previous_target_positions_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> traj_msg_external_point_ptr_;
        bool new_msg_ = false;
        double step_size_ = 0.1;
        rclcpp::Time start_time_;
        std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;

        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            joint_position_command_interface_;        
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            joint_position_state_interface_;
  

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
            command_interface_map_ = {
                {"position", &joint_position_command_interface_}};

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            state_interface_map_ = {
                {"position", &joint_position_state_interface_}};
    };
}

#endif // DYNAARM_POSITION_CONTROLLERS_HPP_