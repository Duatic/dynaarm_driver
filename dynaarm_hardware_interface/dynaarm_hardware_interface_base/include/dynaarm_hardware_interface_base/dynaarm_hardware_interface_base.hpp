// System
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

// ros2_control hardware_interface
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ROS
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

// Urdf
#include "dynaarm_hardware_interface_common/types.hpp"
#include "dynaarm_hardware_interface_common/command_translator.hpp"

namespace dynaarm_hardware_interface_base
{
    class DynaArmHardwareInterfaceBase : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DynaArmHardwareInterfaceBase)

        virtual ~DynaArmHardwareInterfaceBase();

        DynaArmHardwareInterfaceBase() : logger_(rclcpp::get_logger("DynaArmHardwareInterfaceBase"))
        {
            // This is only here otherwise the compiler will complain about the logger var.
            // We initilize the logger in on_init properly
        }

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &system_info);
        virtual hardware_interface::CallbackReturn on_init_derived(const hardware_interface::HardwareInfo &system_info) = 0;

        std::vector<hardware_interface::StateInterface> export_state_interfaces();
        std::vector<hardware_interface::CommandInterface> export_command_interfaces();

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
        virtual hardware_interface::CallbackReturn on_activate_derived(const rclcpp_lifecycle::State &previous_state) = 0;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
        virtual hardware_interface::CallbackReturn on_deactivate_derived(const rclcpp_lifecycle::State &previous_state) = 0;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period);
        virtual void read_motor_states() = 0;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period);
        virtual void write_motor_commands() = 0;

        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/)
        {
            return hardware_interface::return_type::OK;
        }

        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/)
        {
            return hardware_interface::return_type::OK;
        }
        virtual void shutdown() = 0;

    protected:
        rclcpp::Logger logger_;

        std::vector<dynaarm_hardware_interface_common::JointState> joint_state_vector_;
        std::vector<dynaarm_hardware_interface_common::MotorState> motor_state_vector_;

        std::vector<dynaarm_hardware_interface_common::JointCommand> joint_command_vector_;
        std::vector<dynaarm_hardware_interface_common::MotorCommand> motor_command_vector_;

        double command_freeze_mode_{1.0}; // start in freeze mode
    };

}