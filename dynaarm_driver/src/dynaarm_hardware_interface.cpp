#include "dynaarm_driver/dynaarm_hardware_interface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dynaarm_driver {

        hardware_interface::CallbackReturn DynaArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &system_info) {
                //Create a logger with the current hardware name
                const auto name = system_info.name;
                logger = rclcpp::get_logger(name);

                //Init base interface
                if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
                        RCLCPP_FATAL(logger, "Error initialising base interface");
                        return hardware_interface::CallbackReturn::ERROR;
                }
                //Store the system information
                info_ = system_info;

                //Load and parse urdf
                if (!urdf_.initString(info_.original_xml)) {
                        RCLCPP_FATAL(logger, "Error parsing urdf");
                        return hardware_interface::CallbackReturn::ERROR;
                }

                //configure ethercat bus and drives
                const auto ethercat_bus = system_info.hardware_parameters.at("ethercat_bus");
                const ecat_master::EthercatMasterConfiguration ecat_master_config = {.name = name, .networkInterface = ethercat_bus, .timeStep = 0.05};
                //Obtain an instance of the bus from the singleton - if there is no instance it will be created
                ecat_master_ = ecat_master::EthercatMasterSingleton::instance()[ecat_master_config];
                
                //Initialize the state vectors with 0 values
                initializeStateVectors();



                //Every joint refers to a drive
                for(const auto & joint: system_info.joints){
                        const auto address = std::stoi(joint.parameters.at("address"));
                        const auto joint_name = joint.name;
                        auto drive = std::make_shared<anydrive::AnydriveEthercatSlave>(address, joint_name, anydrive::PdoTypeEnum::E);
                        //Store in our internal list so that we can easy refer to them afterwards
                        drives_[joint_name] = drive;
                        //And attach it to the ethercat master
                        ecat_master_->attachDevice(drive);

                        RCLCPP_INFO_STREAM(logger, "Configuring drive: " << joint_name << " at bus address: " << address);

                        //Obtain the current pid gains
                        anydrive::mode::PidGainsF gains;
                        drive->getControlGains(anydrive::mode::ModeEnum::JointPositionVelocityTorquePidGains, gains);

                        //And set the as the currently commanded pid gains
                        auto & pid = command_vectors_.at(joint.name).target_pid;
                        pid.p = gains.getP();
                        pid.i = gains.getI();
                        pid.d = gains.getD();
                }

 

                //Here one might validate the ros2control.xacro joint specifications. I just assume they are fine ;)

                RCLCPP_INFO_STREAM(logger, "Successfully initialized dynaarm hardware interface for: " << name);
                return hardware_interface::CallbackReturn::SUCCESS;
        }

        std::vector<hardware_interface::StateInterface> DynaArmHardwareInterface::export_state_interfaces(){
                 std::vector<hardware_interface::StateInterface> state_interfaces;

                 for(const auto & joint: info_.joints){
                        state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &state_vectors_.at(joint.name).last_position));
                        state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &state_vectors_.at(joint.name).last_velocity));
                        state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_EFFORT, &state_vectors_.at(joint.name).last_torque));

                        state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "temperature_system", &state_vectors_.at(joint.name).last_temperature));
                        state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "temperature_coil_A", &state_vectors_.at(joint.name).last_temperature_coil_A));
                        state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "temperature_coil_B", &state_vectors_.at(joint.name).last_temperature_coil_B));
                        state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "temperature_coil_C", &state_vectors_.at(joint.name).last_temperature_coil_C));
                        
                        state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "bus_voltage", &state_vectors_.at(joint.name).last_bus_voltage));
                        //TODO expose drive state, warnings, imu?
                 }

                 return state_interfaces;
        }
        std::vector<hardware_interface::CommandInterface> DynaArmHardwareInterface::export_command_interfaces(){
                std::vector<hardware_interface::CommandInterface> command_interfaces;

                for(const auto & joint:info_.joints){
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &command_vectors_.at(joint.name).target_position));
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY,  &command_vectors_.at(joint.name).target_velocity));
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_EFFORT,  &command_vectors_.at(joint.name).target_effort));

                        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "p", &command_vectors_.at(joint.name).target_pid.p));
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "i",  &command_vectors_.at(joint.name).target_pid.i));
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "d",  &command_vectors_.at(joint.name).target_pid.d));  
                }
                //Expose a dummy command interface for the freeze mode
                command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.name, "freeze", &command_freeze_mode_));

                return command_interfaces;
        }

        hardware_interface::CallbackReturn DynaArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
                //On activate is already in the realtime loop (on_configure would be in the non_rt loop)

                for(const auto &joint: info_.joints){
                        auto & drive= drives_.at(joint.name);
                        //Log the firmware information of the drive. Might be usefull for debugging issues at customer
                        anydrive::common::BuildInfo info;
                        drive->getBuildInfo(info);
                        RCLCPP_INFO_STREAM(logger, "Drive info: " << joint.name << " Build date: " << info.buildDate << " tag: " << info.gitTag);

                        drive->setFSMGoalState(anydrive::fsm::StateEnum::ControlOp, false,0,0);
                }       

                //Perform a reading once to obtain the current positions
                read(rclcpp::Time(), rclcpp::Duration(std::chrono::nanoseconds(0)));

                for(const auto &joint: info_.joints){
                        command_vectors_[joint.name].target_position = state_vectors_[joint.name].last_position;
                        RCLCPP_INFO_STREAM(logger, "Start position of joint: " << joint.name << " is: " << state_vectors_[joint.name].last_position);
                }
                return hardware_interface::CallbackReturn::SUCCESS;
        }
        hardware_interface::CallbackReturn DynaArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
                for(const auto & joint: info_.joints){
                        auto& drive= drives_.at(joint.name);
                        //TODO I don't know what exactly we want to do here?
                        drive->setFSMGoalState(anydrive::fsm::StateEnum::MotorOp, true, 1.0,0.01);
                        drive->preShutdown(); //At Jan -> I haven't written any lowlevel controller in quite sometime...I guess this is the right way?
                }
                return hardware_interface::CallbackReturn::SUCCESS;
        }
        hardware_interface::CallbackReturn DynaArmHardwareInterface::on_error(const rclcpp_lifecycle::State &previous_state){
                //on_error will be called if read/write return something else than OK
                //This is some workaround I had to include sometime ago because at the moment this is the only save way to stop ros2control if something goes wrong 
                std::exit(-1);
                return hardware_interface::CallbackReturn::FAILURE;
        }

        hardware_interface::return_type DynaArmHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period){
                //read is in the realtime loop
                for(const auto & joint: info_.joints){
                        //Get a reading from the specific drive and 
                        anydrive::ReadingExtended reading;
                        //NOTE: getReading uses a recursive mutex -> It would be better if we could do something like: tryLock and if we can't look then we try again in the next cycle
                        drives_[joint.name]->getReading(reading);  //Use [ ] instead of at for performance reasons

                        //And update the state vector so that controllers can read the current state
                        auto state = reading.getState();

                        auto & state_vector = state_vectors_[joint.name];
                        state_vector.last_position = state.getJointPosition();
                        state_vector.last_velocity = state.getJointVelocity();
                        state_vector.last_torque = state.getJointTorque();

                        state_vector.last_temperature = state.getTemperature();
                        state_vector.last_temperature_coil_A = state.getCoilTemp1();
                        state_vector.last_temperature_coil_B = state.getCoilTemp2();
                        state_vector.last_temperature_coil_C = state.getCoilTemp3();

                        state_vector.last_bus_voltage = state.getVoltage();
                        
                }
                return hardware_interface::return_type::OK;
        }
        hardware_interface::return_type DynaArmHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period){
                //write is in the realtime loop
                for(const auto & joint: info_.joints){
                        //Obtain reference to the specific drive
                        auto & drive = drives_[joint.name]; //Use [ ] instead of at for performance reasons

                        //Only write the command if we are already in the correct state
                        if(drive->goalStateHasBeenReached()){

                                //Convert command vector into an anydrive::Command
                                //Make sure to be in the right mode
                                anydrive::Command cmd;
                                const auto & command_vector = command_vectors_[joint.name];

                                cmd.setJointPosition(command_vector.target_position);
                                cmd.setJointVelocity(command_vector.target_velocity);
                                cmd.setJointTorque(command_vector.target_effort);
                                cmd.setPidGains(anydrive::mode::PidGainsF(command_vector.target_pid.p, command_vector.target_pid.i, command_vector.target_pid.d));
                                cmd.setModeEnum(desired_mode_); //We always fill all command fields but depending on the mode only a subset is used
                                //NOTE: same as with getReading. setCommand uses a recursive mutext -> It would be better if we could doe somethling like: tryLock
                                drive->setCommand(cmd);
                        }
                }
                return hardware_interface::return_type::OK;
        }

        hardware_interface::return_type DynaArmHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces){
                //This is in the non rt loop
                return hardware_interface::return_type::OK;
        }

        hardware_interface::return_type DynaArmHardwareInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces){
                //This is in the rt loop
                //The only thing we want to change here is to switch between freeze mode the JVTPID
                if(start_interfaces.size() == 1 and start_interfaces.at(0) == info_.name + "/freeze"){
                        desired_mode_ = anydrive::mode::ModeEnum::Freeze;
                }
                else {
                        //TODO Check if the controller selects all joints and necessary interfaces
                        //@Jan this will work at the moment if you only have one hardware interface loaded.
                        desired_mode_ = anydrive::mode::ModeEnum::JointPositionVelocityTorquePidGains;
                }
                return hardware_interface::return_type::OK;
        }

        void DynaArmHardwareInterface::initializeStateVectors(){
                for(const auto & joint: info_.joints){
                        state_vectors_[joint.name] = StateVector{};
                        command_vectors_[joint.name] = CommandVector{};
                }
        }

}