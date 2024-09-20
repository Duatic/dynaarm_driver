// System
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"


// ROS
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

//Urdf
#include <urdf/model.h>
#include "dynaarm_driver/command_translator.hpp"
#include "ethercat_sdk_master/EthercatMaster.hpp"
#include <rsl_drive_sdk/Drive.hpp>

namespace dynaarm_driver {
    class DynaArmHardwareInterface: public hardware_interface::SystemInterface 
    {
        public:
            struct PID {
                double p{};
                double i{};
                double d{};
            };
            //I decided to use maps at this point in order to make the mapping easier. It would be more performant to use std::arrays and store the index to name mapping somewhere!
            struct StateVector {
                double last_position{};
                double last_velocity{};
                double last_torque{};
                
                double last_temperature{};
                double last_temperature_coil_A{};
                double last_temperature_coil_B{};
                double last_temperature_coil_C{};

                double last_bus_voltage{};

            };
            struct CommandVector {
                double target_position{};
                double target_velocity{};
                double target_effort{};
                PID target_pid{};
            };

            RCLCPP_SHARED_PTR_DEFINITIONS(DynaArmHardwareInterface)
            
            virtual ~DynaArmHardwareInterface();

            DynaArmHardwareInterface(): logger(rclcpp::get_logger("dynaarm_hardware_interface")){
                //This is only here otherwise the compiler will complain about the logger var.
                //We initilize the logger in on_init properly
            }
            
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &system_info);

            std::vector<hardware_interface::StateInterface> export_state_interfaces();
            std::vector<hardware_interface::CommandInterface> export_command_interfaces();

            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
            hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);


            hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period);
            hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period);

            hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces);

            hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces);

        private:

            rclcpp::Logger logger;
            urdf::Model urdf_;

            ecat_master::EthercatMaster::SharedPtr ecat_master_;            
            std::map<std::string, rsl_drive_sdk::DriveEthercatDevice::SharedPtr> drives_;            

            std::atomic<bool> startupAbortFlag_{false};
            std::atomic<bool> abrtFlag_{false};
            std::unique_ptr<std::thread> shutdownWorkerThread_;

            CommandTranslator command_translator_;
            Eigen::Vector4d state_joint_positions_;
            Eigen::Vector4d state_joint_velocities_;
            Eigen::Vector4d state_joint_torques_;
            Eigen::Vector4d command_joint_positions_;
            Eigen::Vector4d command_joint_velocities_;
            Eigen::Vector4d command_joint_torques_;
            std::map<std::string, StateVector> state_vectors_;
            std::map<std::string, CommandVector> command_vectors_;
            void initializeStateVectors();
            void shutdown();

            double command_freeze_mode_{};
            rsl_drive_sdk::mode::ModeEnum desired_mode_{rsl_drive_sdk::mode::ModeEnum::Freeze};
    };  


}