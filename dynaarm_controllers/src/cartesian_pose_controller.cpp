/*
 * Copyright 2025 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

#include <dynaarm_controllers/cartesian_pose_controller.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <pinocchio/collision/collision.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/check-data.hpp>

namespace dynaarm_controllers
{

bool computeIK(const pinocchio::Model& model, pinocchio::Data& data, const pinocchio::SE3& target_pose,
               const Eigen::VectorXd& q_in, const pinocchio::JointIndex joint_id, Eigen::VectorXd& q_out)
{
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();

  // Create a copy if the input data that we can work on
  Eigen::VectorXd q = q_in;

  for (std::size_t i = 0; i < IT_MAX; i++) {
    // Compute the forward kinematics with the current configuration and check if it is close enough to where we want
    pinocchio::forwardKinematics(model, data, q);
    const pinocchio::SE3 iMd = data.oMi[joint_id].actInv(target_pose);
    Eigen::Matrix<double, 6, 1> err = log6(iMd).toVector();

    if (err.norm() < eps) {
      q_out = q;
      return true;
    }

    pinocchio::computeJointJacobian(model, data, q, joint_id, J);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;

    Eigen::VectorXd v(model.nv);
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);

    // if (!(i % 10))
    //   std::cout << i << ": error = " << err.transpose() << std::endl;
  }

  // We where not successful - set the output to the original input. Which avoid accidental motions
  q_out = q_in;
  return false;
}

pinocchio::SE3 rosPoseToSE3(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
  return pinocchio::SE3(q.normalized(), t);
}

CartesianPoseController::CartesianPoseController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration CartesianPoseController::command_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

controller_interface::InterfaceConfiguration CartesianPoseController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.emplace_back(joint + "/" + "acceleration_commanded");
  }

  return config;
}

controller_interface::CallbackReturn CartesianPoseController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<cartesian_pose_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // check if joints are empty
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  try {
    // 1. build the pinocchio model from the urdf
    RCLCPP_DEBUG(get_node()->get_logger(), "Building Pinocchio model from URDF...");
    pinocchio::urdf::buildModelFromXML(get_robot_description(), pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    RCLCPP_DEBUG(get_node()->get_logger(), "Pinocchio model built with %zu joints", pinocchio_model_.joints.size() - 1);

    // 2. Build the collision model from urdf and srdf (only if SRDF is provided)
    if (!params_.srdf.empty()) {
      RCLCPP_DEBUG(get_node()->get_logger(), "Building collision geometry...");
      std::stringstream urdf_stream;
      urdf_stream << get_robot_description();
      pinocchio::urdf::buildGeom(pinocchio_model_, urdf_stream, pinocchio::COLLISION, pinocchio_geom_);

      pinocchio_geom_.addAllCollisionPairs();
      pinocchio::srdf::removeCollisionPairsFromXML(pinocchio_model_, pinocchio_geom_, params_.srdf);
      RCLCPP_DEBUG(get_node()->get_logger(), "Collision geometry built with %zu collision pairs", pinocchio_geom_.collisionPairs.size());
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "No SRDF provided - collision checking will be disabled");
    }

    // 3. Validate that all controller joints exist in the Pinocchio model
    RCLCPP_DEBUG(get_node()->get_logger(), "Validating controller joints...");
    std::vector<pinocchio::JointIndex> controller_joint_indices;
    for (const auto& joint_name : params_.joints) {
      if (!pinocchio_model_.existJointName(joint_name)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      auto joint_id = pinocchio_model_.getJointId(joint_name);
      controller_joint_indices.push_back(joint_id);
    }

    // 4. Validate kinematic chain structure (only if more than one joint)
    if (params_.joints.size() > 1) {
      RCLCPP_DEBUG(get_node()->get_logger(), "Validating kinematic chain structure...");
      for (size_t i = 1; i < controller_joint_indices.size(); ++i) {
        auto current_joint_id = controller_joint_indices[i];
        auto previous_joint_id = controller_joint_indices[i-1];
        
        // Check if current joint is a descendant of the previous joint in the kinematic tree
        bool is_valid_chain = false;
        auto parent_id = pinocchio_model_.parents[current_joint_id];
        
        // Traverse up the kinematic tree to see if we find the previous joint
        while (parent_id != 0) {
          if (parent_id == previous_joint_id) {
            is_valid_chain = true;
            break;
          }
          parent_id = pinocchio_model_.parents[parent_id];
        }
        
        if (!is_valid_chain) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Invalid kinematic chain: Joint '%s' (index %zu) is not a descendant of joint '%s' (index %zu) in the kinematic tree.",
                       params_.joints[i].c_str(), i, params_.joints[i-1].c_str(), i-1);
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Please check that the 'joints' parameter lists the joints in the correct kinematic order.");
          return controller_interface::CallbackReturn::ERROR;
        }
      }
    }

    // 5. Validate end effector frame exists
    if (!pinocchio_model_.existFrame(params_.end_effector_frame)) {
      RCLCPP_ERROR(get_node()->get_logger(), "End effector frame '%s' not found in Pinocchio model.", params_.end_effector_frame.c_str());
      
      // Debug: List all available frames
      RCLCPP_ERROR(get_node()->get_logger(), "Available frames in Pinocchio model:");
      for (size_t i = 0; i < pinocchio_model_.frames.size(); ++i) {
        RCLCPP_ERROR(get_node()->get_logger(), "  [%zu]: %s", i, pinocchio_model_.frames[i].name.c_str());
      }
      
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), 
                "Successfully configured controller with %zu joints and end effector frame '%s'", 
                params_.joints.size(), params_.end_effector_frame.c_str());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during Pinocchio model setup: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Setup the pose listener
  pose_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", 10, [&](const geometry_msgs::msg::PoseStamped& msg) {
        buffer_pose_cmd_.writeFromNonRT(msg);

        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "New pose target: " << msg.pose.position.x << ", "
                                                                         << msg.pose.position.y << ", "
                                                                         << msg.pose.position.z);
      });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = true;

  // clear out vectors in case of restart
  joint_position_command_interfaces_.clear();
  joint_velocity_command_interfaces_.clear();

  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  joint_acceleration_state_interfaces_.clear();

  // get the actual interface in an ordered way (same order as the joints parameter)
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, hardware_interface::HW_IF_POSITION, joint_position_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, hardware_interface::HW_IF_VELOCITY, joint_velocity_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - velocity");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "acceleration_commanded",
                                                    joint_acceleration_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - acceleration");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints,
                                                    hardware_interface::HW_IF_POSITION,
                                                    joint_position_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints,
                                                    hardware_interface::HW_IF_VELOCITY,
                                                    joint_velocity_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                  [[maybe_unused]] const rclcpp::Duration& period)
{
  using namespace pinocchio;
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE || !active_) {
    return controller_interface::return_type::OK;
  }

  const std::size_t joint_count = joint_position_state_interfaces_.size();

  // Build full-size vectors for all robot joints (Pinocchio expects this)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  // Map: Pinocchio joint name -> index in q/v
  for (std::size_t i = 0; i < joint_count; i++) {
    const std::string& joint_name = params_.joints[i];
    auto idx = pinocchio_model_.getJointId(joint_name);
    if (idx == 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
      return controller_interface::return_type::ERROR;
    }
    // Pinocchio joint index starts at 1, q/v index is idx-1
    q[pinocchio_model_.joints[idx].idx_q()] = joint_position_state_interfaces_.at(i).get().get_value();
    v[pinocchio_model_.joints[idx].idx_v()] = joint_velocity_state_interfaces_.at(i).get().get_value();
    a[pinocchio_model_.joints[idx].idx_v()] = joint_acceleration_state_interfaces_.at(i).get().get_value();
  }

  pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(params_.end_effector_frame);

  const pinocchio::JointIndex joint_id = pinocchio_model_.frames[frame_id].parent;

  const auto target_pose = rosPoseToSE3(buffer_pose_cmd_.readFromRT()->pose);

  Eigen::VectorXd q_out = Eigen::VectorXd::Zero(pinocchio_model_.nq);

  pinocchio_data_ = pinocchio::Data(pinocchio_model_);
  if (!computeIK(pinocchio_model_, pinocchio_data_, target_pose, q, joint_id, q_out)) {
    RCLCPP_ERROR_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Failed to compute IK");
  }

  pinocchio::GeometryData geom_data(pinocchio_geom_);
  const auto collides =
      pinocchio::computeCollisions(pinocchio_model_, pinocchio_data_, pinocchio_geom_, geom_data, q_out);

  if (!collides) {
    for (std::size_t i = 0; i < joint_count; i++) {
      const std::string& joint_name = params_.joints[i];
      auto idx = pinocchio_model_.getJointId(joint_name);
      if (idx == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
        return controller_interface::return_type::ERROR;
      }
      // Pinocchio joint index starts at 1, q/v index is idx-1

      joint_position_command_interfaces_.at(i).get().set_value<double>(q_out[pinocchio_model_.joints[idx].idx_q()]);
    }
  }
  else {
    // Print the status of all the collision pairs
    for(size_t k = 0; k < pinocchio_geom_.collisionPairs.size(); ++k)
    {
      const CollisionPair & cp = pinocchio_geom_.collisionPairs[k];
      const hpp::fcl::CollisionResult & cr = geom_data.collisionResults[k];
      
      std::cout << "collision pair: " << cp.first << " , " << cp.second << " - collision: ";
      std::cout << (cr.isCollision() ? "yes" : "no") << std::endl;
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
CartesianPoseController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_error([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace dynaarm_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::CartesianPoseController, controller_interface::ControllerInterface)
