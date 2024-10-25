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
        //Some convienent typedef for easier handling of messages
        using ArmState = dynaarm_msgs::msg::ArmState;
        using ArmStatePublisher = realtime_tools::RealtimePublisher<ArmState>;
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

    private:
        //Access to controller parameters via generate_parameter_library
        std::unique_ptr<dynaarm_status_controller::ParamListener> param_listener_;
        dynaarm_status_controller::Params params_;

        //The actual state publisher and it's realtime wrapper
        rclcpp::Publisher<ArmState>::SharedPtr arm_state_pub_;
        std::unique_ptr<ArmStatePublisher> arm_state_pub_rt_;

        //State interface references 
        StateInterfaceReferences joint_position_interfaces_;
        StateInterfaceReferences joint_velocity_interfaces_;
        StateInterfaceReferences joint_effort_interfaces_;
        StateInterfaceReferences joint_temperature_system_interfaces_;
        StateInterfaceReferences joint_temperature_phase_a_interfaces_;
        StateInterfaceReferences joint_temperature_phase_b_interfaces_;
        StateInterfaceReferences joint_temperature_phase_c_interfaces_;
        StateInterfaceReferences joint_bus_voltage_interfaces_;
        //TODO add additional interfaces


  
    };
}