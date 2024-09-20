#include "dynaarm_driver/dynaarm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dynaarm_driver
{
    DynaArmHardwareInterface::~DynaArmHardwareInterface()
    {
        // If controller manager is shutdown via Ctrl + C, the on_deactivate methods won't be called.
        // We need to call them here to ensure that the device is stopped and disconnected.
        shutdown();
    }

    hardware_interface::CallbackReturn DynaArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &system_info)
    {
        // Create a logger with the current hardware name
        const auto name = system_info.name;
        logger = rclcpp::get_logger(name);

        // Init base interface
        if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_FATAL(logger, "Error initialising base interface");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Store the system information
        info_ = system_info;

        // Load and parse urdf
        if (!urdf_.initString(info_.original_xml))
        {
            RCLCPP_FATAL(logger, "Error parsing urdf");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // configure ethercat bus and drives
        const auto ethercat_bus = system_info.hardware_parameters.at("ethercat_bus");
        const ecat_master::EthercatMasterConfiguration ecat_master_config = {.name = name, .networkInterface = ethercat_bus, .timeStep = 0.001}; //TODO set timestep according to the update rate of ros2control (or spin asynchronously)
        
        // Obtain an instance of the bus from the singleton - if there is no instance it will be created
        ecat_master_ = std::make_shared<ecat_master::EthercatMaster>();
        ecat_master_->loadEthercatMasterConfiguration(ecat_master_config);

        // Initialize the state vectors with 0 values
        initializeStateVectors();

        // Every joint refers to a drive
        for (const auto &joint : system_info.joints)
        {
            const auto address = std::stoi(joint.parameters.at("address"));
            const auto joint_name = joint.name;

            const std::string device_file_path = system_info.hardware_parameters.at("drive_config_file");            
            auto drive = rsl_drive_sdk::DriveEthercatDevice::deviceFromFile(device_file_path, joint_name, address, rsl_drive_sdk::PdoTypeEnum::C);

            // Store in our internal list so that we can easy refer to them afterwards            
            drives_[joint_name] = drive;
            
            // And attach it to the ethercat master
            if (ecat_master_->attachDevice(drive) == false)
            {
                RCLCPP_ERROR_STREAM(logger, "Could not attach the slave drive to the master.");
            }

            RCLCPP_INFO_STREAM(logger, "Configuring drive: " << joint_name << " at bus address: " << address);            
        }

        if (ecat_master_->startup(startupAbortFlag_) == false)
        {
            RCLCPP_ERROR_STREAM(logger, "Could not start the Ethercat Master.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO_STREAM(logger, "Successfully started Ethercat Master on Network Interface: "
                                       << ecat_master_->getBusPtr()->getName());

        RCLCPP_INFO_STREAM(logger, "Successfully initialized dynaarm hardware interface for: " << name);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DynaArmHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (const auto &joint : info_.joints)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &state_vectors_.at(joint.name).last_position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &state_vectors_.at(joint.name).last_velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_EFFORT, &state_vectors_.at(joint.name).last_torque));

            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "temperature_system", &state_vectors_.at(joint.name).last_temperature));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "temperature_coil_A", &state_vectors_.at(joint.name).last_temperature_coil_A));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "temperature_coil_B", &state_vectors_.at(joint.name).last_temperature_coil_B));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "temperature_coil_C", &state_vectors_.at(joint.name).last_temperature_coil_C));

            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "bus_voltage", &state_vectors_.at(joint.name).last_bus_voltage));
            
            // TODO expose drive state, warnings, imu?
        }

        return state_interfaces;
    }
    std::vector<hardware_interface::CommandInterface> DynaArmHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (const auto &joint : info_.joints)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &command_vectors_.at(joint.name).target_position));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &command_vectors_.at(joint.name).target_velocity));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_EFFORT, &command_vectors_.at(joint.name).target_effort));

            command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "p", &command_vectors_.at(joint.name).target_pid.p));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "i", &command_vectors_.at(joint.name).target_pid.i));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "d", &command_vectors_.at(joint.name).target_pid.d));
        }
        // Expose a dummy command interface for the freeze mode
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.name, "freeze", &command_freeze_mode_));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DynaArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // On activate is already in the realtime loop (on_configure would be in the non_rt loop)
        for (const auto &joint : info_.joints)
        {
            auto &drive = drives_[joint.name];            

            
            // Put into controlOP, in blocking mode.            
            drive->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 1, 10);

            // Log the firmware information of the drive. Might be usefull for debugging issues at customer
            rsl_drive_sdk::common::BuildInfo info;
            drive->getBuildInfo(info);
            RCLCPP_INFO_STREAM(logger, "Drive info: " << joint.name << " Build date: " << info.buildDate << " tag: " << info.gitTag);
        }

        if (ecat_master_->setRealtimePriority(48) == false)
        {
            RCLCPP_WARN_STREAM(logger, "Could not incrase thread priority - check user privileges.");
        }

        if (ecat_master_->activate())
        {
            RCLCPP_INFO_STREAM(logger, "Activated the Bus: " << ecat_master_->getBusPtr()->getName());
        }

        // Perform a reading once to obtain the current positions
        read(rclcpp::Time(), rclcpp::Duration(std::chrono::nanoseconds(0)));

        for (const auto &joint : info_.joints)
        {    
            command_vectors_[joint.name].target_position = state_vectors_[joint.name].last_position;
            RCLCPP_INFO_STREAM(logger, "Start position of joint: " << joint.name << " is: " << state_vectors_[joint.name].last_position);            
            auto &drive = drives_[joint.name];      


            //In case wie are in error state clear the error and try again
            rsl_drive_sdk::ReadingExtended reading;
            drive->getReading(reading);

            if(reading.getState().getStatusword().getStateEnum() == rsl_drive_sdk::fsm::StateEnum::Error){
                drive->setControlword(RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY);
                drive->updateWrite();
                 drive->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 1, 10);
            }

        }



        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynaArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {

        for (const auto &joint : info_.joints)
        {
            auto &drive = drives_.at(joint.name);            
            drive->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 3.0, 0.01);            
            
            rsl_drive_sdk::Command cmd;
            cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::Freeze);
            drive->setCommand(cmd);            
        }



        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynaArmHardwareInterface::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        shutdown();
        return hardware_interface::CallbackReturn::FAILURE;
    }

    hardware_interface::return_type DynaArmHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // The ethercat update is the first in the read function to directly get all controller states
        ecat_master_->update(ecat_master::UpdateMode::NonStandalone);

        for (size_t i = 0; i < info_.joints.size(); ++i) 
        {
            std::string joint_name = info_.joints[i].name;            

            // Get a reading from the specific drive and
            rsl_drive_sdk::ReadingExtended reading;
            // NOTE: getReading uses a recursive mutex -> It would be better if we could do something like: tryLock and if we can't look then we try again in the next cycle
            drives_[joint_name]->getReading(reading); // Use [ ] instead of at for performance reasons

            // And update the state vector so that controllers can read the current state
            auto state = reading.getState();

            auto &state_vector = state_vectors_[joint_name];

            state_joint_positions_[i] = state.getJointPosition(); 
            state_joint_velocities_[i] = state.getJointVelocity(); 
            state_joint_torques_[i] = state.getJointTorque(); 
            
            state_vector.last_temperature = state.getTemperature();
            state_vector.last_temperature_coil_A = state.getCoilTemp1();
            state_vector.last_temperature_coil_B = state.getCoilTemp2();
            state_vector.last_temperature_coil_C = state.getCoilTemp3();

            state_vector.last_bus_voltage = state.getVoltage();
        }

        // Transform the joint positions using the matrix transformation
        Eigen::Vector4d transformed_positions = command_translator_.mapFromDynaarmToSerialCoordinates(state_joint_positions_);
        Eigen::Vector4d transformed_velocities = command_translator_.mapFromDynaarmToSerialCoordinates(state_joint_velocities_);
        Eigen::Vector4d transformed_torques = command_translator_.mapFromDynaarmToSerialTorques(state_joint_torques_);
        
        for (size_t i = 0; i < info_.joints.size(); ++i) 
        {
            std::string joint_name = info_.joints[i].name;
            state_vectors_[joint_name].last_position = transformed_positions[i];
            state_vectors_[joint_name].last_velocity = transformed_velocities[i];
            state_vectors_[joint_name].last_torque = transformed_torques[i];
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DynaArmHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {       
        for (size_t i = 0; i < info_.joints.size(); ++i) 
        {
            std::string joint_name = info_.joints[i].name;
            const auto & command_vector = command_vectors_[joint_name];

            command_joint_positions_[i] = command_vector.target_position; 
            command_joint_velocities_[i] = command_vector.target_velocity; 
            command_joint_torques_[i] = command_vector.target_effort; 
        }
        // Transform the joint positions using the matrix transformation
        Eigen::Vector4d transformed_positions = command_translator_.mapFromSerialToDynaarmCoordinates(command_joint_positions_);
        Eigen::Vector4d transformed_velocities = command_translator_.mapFromSerialToDynaarmCoordinates(command_joint_velocities_);
        Eigen::Vector4d transformed_torques = command_translator_.mapFromSerialToDynaarmTorques(command_joint_torques_);

        for (size_t i = 0; i < info_.joints.size(); ++i) 
        {
            std::string joint_name = info_.joints[i].name;

            // Obtain reference to the specific drive
            auto &drive = drives_[joint_name];
                        
            //Only write the command if we are already in the correct state
            if(drive->goalStateHasBeenReached())
            {
                //Convert command vector into an rsl_drive_sdk::Command
                //Make sure to be in the right mode
                rsl_drive_sdk::Command cmd;
                const auto & command_vector = command_vectors_[joint_name];

                rsl_drive_sdk::mode::PidGainsF gains;
                gains.setP(command_vector.target_pid.p);
                gains.setI(command_vector.target_pid.i);
                gains.setD(command_vector.target_pid.d);
                
                //std::cout << joint_name << " P:" << gains.getP() << " I:" << gains.getI() << " D:" << gains.getD() << std::endl;
                //std::cout << joint_name << ": " << command_vector.target_effort << std::endl;         

                // if (joint_name == "FA_ROT")
                // {
                //     std::cout << joint_name << ": " << transformed_positions[i] << " " << transformed_velocities[i] << " " << transformed_torques[i] << std::endl;
                // }                
                
                // std::cout << "Mode: " << desired_mode_ << std::endl;

                cmd.setJointPosition(transformed_positions[i]);
                cmd.setJointVelocity(transformed_velocities[i]);
                cmd.setJointTorque(transformed_torques[i]);
                cmd.setPidGains(gains);
                cmd.setModeEnum(desired_mode_);
                
                //We always fill all command fields but depending on the mode only a subset is used                
                drive->setCommand(cmd);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DynaArmHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
    {
        // This is in the non rt loop
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DynaArmHardwareInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
    {
        // This is in the rt loop
        // The only thing we want to change here is to switch between freeze mode the JVTPID
        if (start_interfaces.size() == 1 and start_interfaces.at(0) == info_.name + "/freeze")
        {
            desired_mode_ = rsl_drive_sdk::mode::ModeEnum::Freeze;
        }
        else
        {
            // TODO Check if the controller selects all joints and necessary interfaces
            //@Jan this will work at the moment if you only have one hardware interface loaded.
            desired_mode_ = rsl_drive_sdk::mode::ModeEnum::JointPositionVelocityTorquePidGains;

            RCLCPP_INFO_STREAM(logger, "Mode: " << desired_mode_);

            for (size_t i = 0; i < info_.joints.size(); ++i)
            {
                std::string joint_name = info_.joints[i].name;

                rsl_drive_sdk::mode::PidGainsF gains;
                drives_[joint_name]->getControlGains(desired_mode_, gains);

                command_vectors_[joint_name].target_pid.p = gains.getP();
                command_vectors_[joint_name].target_pid.i = gains.getI();
                command_vectors_[joint_name].target_pid.d = gains.getD();
                //std::cout << joint_name << command_vectors_[joint_name].target_pid.p << std::endl;                      
                //std::cout << "P: " << gains.getP() << " I: " << gains.getI() << " D: " << gains.getD() << std::endl;
            }

            // desired_mode_ = rsl_drive_sdk::mode::ModeEnum::Disable;
        }
        return hardware_interface::return_type::OK;
    }

    void DynaArmHardwareInterface::shutdown()
    {
        shutdownWorkerThread_ = std::make_unique<std::thread>([this]() -> void
        {            
            // here the watchdog on the slave is activated. therefore don't block/sleep for 100ms..
            while (!abrtFlag_) 
            {
                ecat_master_->update(ecat_master::UpdateMode::StandaloneEnforceStep);            
            }

            // make sure that bus is in SAFE_OP state, if preShutdown(true) should already do it, but makes sense to have this call here.
            ecat_master_->deactivate(); 
        });

        // call preShutdown before terminating the cyclic PDO communication!!
        if (ecat_master_)
        {
            ecat_master_->preShutdown(true);
        }
        RCLCPP_INFO_STREAM(logger,"PreShutdown ethercat master and all slaves.");

        abrtFlag_ = true;
        if (shutdownWorkerThread_)
        {
            if (shutdownWorkerThread_->joinable())
            {
            shutdownWorkerThread_->join();
            }
        }
        if (ecat_master_)
        {
            ecat_master_->shutdown();
        }
        RCLCPP_INFO_STREAM(logger, "Fully shutdown.");
    }

    void DynaArmHardwareInterface::initializeStateVectors()
    {
        for (const auto &joint : info_.joints)
        {
            state_vectors_[joint.name] = StateVector{};
            command_vectors_[joint.name] = CommandVector{};
        }
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynaarm_driver::DynaArmHardwareInterface, hardware_interface::SystemInterface)