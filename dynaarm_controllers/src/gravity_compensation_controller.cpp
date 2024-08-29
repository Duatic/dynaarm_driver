#include "dynaarm_controllers/gravity_compensation_controller.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using config_type = controller_interface::interface_configuration_type;

namespace dynaarm_controllers
{
    GravityCompensationController::GravityCompensationController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn GravityCompensationController::on_init()
    {
        // should have error handling
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
        
        std::string urdf_string = get_robot_description();

        pinocchio::urdf::buildModelFromXML(urdf_string, model);
        data = pinocchio::Data(model);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration GravityCompensationController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        conf.names.reserve(joint_names_.size() * command_interface_types_.size());

        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : command_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);                
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration GravityCompensationController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());

        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type GravityCompensationController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        Eigen::VectorXd q = pinocchio::neutral(model);
        // convert ROS joint config to pinocchio config
        for (int i = 0; i < model.nv; i++)
        {
            int jidx = model.getJointId(model.names[i + 1]);
            int qidx = model.idx_qs[jidx];
            q[qidx] = joint_position_state_interface_[i].get().get_value();
        }

        Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model, data, q);
        std::cout << std::fixed << std::setprecision(10);
        // add gravity compensation torque to base command
        for (int i = 0; i < model.nv; i++)
        {   
            double new_effort = gravity[i] * 1.0;

            if (i == 1) 
            {
                //std::cout << i << " - NEW: " << new_effort << std::endl;     
                joint_effort_command_interface_[i].get().set_value(new_effort);
            }            
        }

        // for (size_t i = 0; i < joint_p_command_interface_.size(); ++i)
        // {
        //     std::cout << i << std::endl;
        //     joint_p_command_interface_[i].get().set_value(0.0);
        //     joint_i_command_interface_[i].get().set_value(0.0);
        //     joint_d_command_interface_[i].get().set_value(0.0);
        // }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // clear out vectors in case of restart
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_effort_command_interface_.clear();
        joint_p_command_interface_.clear();
        joint_i_command_interface_.clear();
        joint_d_command_interface_.clear();

        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();
        joint_effort_state_interface_.clear();

        // assign command interfaces
        for (auto &interface : command_interfaces_)
        {
            command_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        // assign state interfaces
        for (auto &interface : state_interfaces_)
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }        

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }
} // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::GravityCompensationController,
                       controller_interface::ControllerInterface)