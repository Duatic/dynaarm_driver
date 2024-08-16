#include "dynaarm_controllers/position_controller.hpp"
#include <cmath>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using config_type = controller_interface::interface_configuration_type;

namespace dynaarm_controllers
{
    PositionController::PositionController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn PositionController::on_init()
    {
        clock_ = rclcpp::Clock(RCL_ROS_TIME); // Use ROS Time
        rclcpp::Time current_time = clock_.now();
        previous_time_ = std::chrono::nanoseconds(current_time.nanoseconds());

        // should have error handling
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PositionController::command_interface_configuration() const
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

    controller_interface::InterfaceConfiguration PositionController::state_interface_configuration() const
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

    controller_interface::CallbackReturn PositionController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        auto callback = [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
        {
            traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
            new_msg_ = true;
        };

        joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>("/position_controller/commands", rclcpp::SystemDefaultsQoS(), callback);
        return CallbackReturn::SUCCESS;
    }

    double PositionController::interpolateNextStep(double current_position,
                                                   double target_position,
                                                   double &step_size,
                                                   double max_velocity,
                                                   double delta_time_sec)
    {
        // Calculate the difference between the target and current positions
        double position_difference = target_position - current_position;

        // Calculate the maximum possible step size based on the maximum velocity and PI
        double max_step_size = max_velocity * delta_time_sec * M_PI;

        // If the step size provided is greater than the calculated max step size, limit it
        if (std::fabs(step_size) > max_step_size) {
            step_size = (step_size > 0) ? max_step_size : -max_step_size;
        }

        // If the difference is smaller than the step size, set the current position to the target position
        if (std::fabs(position_difference) <= std::fabs(step_size)) {
            step_size = position_difference; // Move directly to target_position
            return target_position;
        }

        // Calculate the direction of the step
        double direction = (position_difference > 0) ? 1.0 : -1.0;

        // Adjust the step size to ensure it reaches the maximum velocity if possible
        step_size = direction * std::min(max_step_size, std::fabs(position_difference));

        // Calculate the next position step
        return current_position + step_size;
    }

    controller_interface::return_type PositionController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (new_msg_)
        {
            trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();            
        }

        new_msg_ = false;

        if (trajectory_msg_ == nullptr)
        {
            return controller_interface::return_type::OK;
        }

        // Get the current time
        rclcpp::Time current_time = clock_.now(); // Or time_source.now()
        auto current_time_ns = std::chrono::nanoseconds(current_time.nanoseconds());
        // Convert delta_time to seconds
        auto delta_time_ns = current_time_ns - previous_time_;
        double delta_time_sec = delta_time_ns.count() * 1e-9;
        // Update previous_time for the next call
        previous_time_ = current_time_ns;

        for (size_t i = 0; i < joint_position_state_interface_.size(); i++)
        {   
            double current_position = previous_target_positions_[i];
            const auto &point = trajectory_msg_->points[0];
            double target_position = point.positions[i];
            double step_size = point.velocities[i];
            double max_velocity = 0.5;    
            double next_position = interpolateNextStep(current_position, target_position, step_size, max_velocity, delta_time_sec);
            previous_target_positions_[i] = next_position;
            //std::cout << " STEP:" << step_size << std::endl;
            //std::cout << "LAST PREV TAR: " << previous_target_positions_[i] << std::endl;

            joint_position_command_interface_[i].get().set_value(target_position);
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn PositionController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // clear out vectors in case of restart
        joint_position_command_interface_.clear();               
        joint_position_state_interface_.clear();
        previous_target_positions_.clear();

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

        previous_target_positions_.reserve(joint_position_state_interface_.size());  // Reserve space if you know the size
        for (size_t i = 0; i < joint_position_state_interface_.size(); i++)
        {
            previous_target_positions_.push_back(joint_position_state_interface_[i].get().get_value());
            std::cout << i << " START POS: " << previous_target_positions_[i] << std::endl;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PositionController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PositionController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PositionController::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PositionController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }
} // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::PositionController,
                       controller_interface::ControllerInterface)