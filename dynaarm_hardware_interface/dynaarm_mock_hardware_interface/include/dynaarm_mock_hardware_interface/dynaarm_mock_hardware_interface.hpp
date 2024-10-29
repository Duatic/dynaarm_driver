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
#include <ament_index_cpp/get_package_share_directory.hpp>

// hardware interface
#include "dynaarm_hardware_interface_base/dynaarm_hardware_interface_base.hpp"

namespace dynaarm_mock_hardware_interface
{
    class DynaarmMockHardwareInterface : public dynaarm_hardware_interface_base::DynaArmHardwareInterfaceBase
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DynaarmMockHardwareInterface)

        hardware_interface::CallbackReturn on_init_derived(const hardware_interface::HardwareInfo &system_info) override;

        hardware_interface::CallbackReturn on_activate_derived(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate_derived(const rclcpp_lifecycle::State &previous_state) override;

        void read_motor_states() override;
        void write_motor_commands() override;

        void shutdown() override;
    };

}