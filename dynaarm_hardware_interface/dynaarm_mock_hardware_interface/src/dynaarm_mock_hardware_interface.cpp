#include "dynaarm_mock_hardware_interface/dynaarm_mock_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dynaarm_mock_hardware_interface
{
    hardware_interface::CallbackReturn DynaarmMockHardwareInterface::on_init_derived(const hardware_interface::HardwareInfo & /*system_info*/)
    {
        RCLCPP_INFO_STREAM(logger_, "Successfully initialized dynaarm hardware interface for DynaarmMockHardwareInterface");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynaarmMockHardwareInterface::on_activate_derived(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO_STREAM(logger_, "Successfully activated dynaarm hardware interface for DynaarmMockHardwareInterface");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynaarmMockHardwareInterface::on_deactivate_derived(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO_STREAM(logger_, "Successfully deactivated dynaarm hardware interface for DynaarmMockHardwareInterface");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void DynaarmMockHardwareInterface::read_motor_states()
    {
        for (int i = 0; i < static_cast<int>(info_.joints.size()); i++)
        {
            motor_state_vector_[i].position = motor_command_vector_[i].position;
            motor_state_vector_[i].velocity = motor_command_vector_[i].velocity;
            motor_state_vector_[i].effort = motor_command_vector_[i].effort;

            motor_state_vector_[i].temperature = 0.0;
            motor_state_vector_[i].temperature_coil_A = 0.0;
            motor_state_vector_[i].temperature_coil_B = 0.0;
            motor_state_vector_[i].temperature_coil_C = 0.0;
            motor_state_vector_[i].bus_voltage = 0.0;
        }
    }

    void DynaarmMockHardwareInterface::write_motor_commands() {}

    void DynaarmMockHardwareInterface::shutdown() {}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_mock_hardware_interface::DynaarmMockHardwareInterface, hardware_interface::SystemInterface)