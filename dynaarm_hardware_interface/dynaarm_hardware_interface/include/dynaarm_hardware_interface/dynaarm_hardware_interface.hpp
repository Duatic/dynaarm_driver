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

// sdk
#include "ethercat_sdk_master/EthercatMaster.hpp"
#include <rsl_drive_sdk/Drive.hpp>

namespace dynaarm_hardware_interface
{
    class DynaArmHardwareInterface : public dynaarm_hardware_interface_base::DynaArmHardwareInterfaceBase
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DynaArmHardwareInterface)

        hardware_interface::CallbackReturn on_init_derived(const hardware_interface::HardwareInfo &system_info) override;

        hardware_interface::CallbackReturn on_activate_derived(const rclcpp_lifecycle::State &previous_state);
        hardware_interface::CallbackReturn on_deactivate_derived(const rclcpp_lifecycle::State &previous_state);

        void read_motor_states() override;
        void write_motor_commands() override;

        void shutdown() override;

    private:
        ecat_master::EthercatMaster::SharedPtr ecat_master_;
        std::vector<rsl_drive_sdk::DriveEthercatDevice::SharedPtr> drives_;

        std::atomic<bool> startupAbortFlag_{false};
        std::atomic<bool> abrtFlag_{false};
        std::unique_ptr<std::thread> shutdownWorkerThread_;
    };

}