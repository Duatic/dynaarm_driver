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

// C++ system headers
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

#include <pinocchio/collision/collision.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/check-data.hpp>

// Other headers
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace dynaarm_controllers
{

bool computeIK(const pinocchio::Model& model, pinocchio::Data& data, const pinocchio::SE3& target_pose,
               const Eigen::VectorXd& q_in, const pinocchio::FrameIndex frame_id, Eigen::VectorXd& q_out)
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
    pinocchio::updateFramePlacements(model, data);
    const pinocchio::SE3 iMf = data.oMf[frame_id].actInv(target_pose);
    Eigen::Matrix<double, 6, 1> err = log6(iMf).toVector();

    if (err.norm() < eps) {
      q_out = q;
      return true;
    }

    // Use frame Jacobian for the specified frame so the IK is solved at the end-effector frame
    pinocchio::computeFrameJacobian(model, data, q, frame_id, J);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMf.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;

    Eigen::VectorXd v(model.nv);
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);
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

std::pair<pinocchio::SE3, bool> transformPoseToModelBaseFrame(const geometry_msgs::msg::PoseStamped& pose_msg,
                                                              const pinocchio::SE3& target_pose,
                                                              const pinocchio::Model& model, pinocchio::Data& data,
                                                              const Eigen::VectorXd& q_current, rclcpp::Logger logger)
{
  // If no frame_id specified or it's "world", assume pose is already in model base frame
  if (pose_msg.header.frame_id.empty() || pose_msg.header.frame_id == "world" || pose_msg.header.frame_id == "") {
    return { target_pose, true };
  }

  // Get the transformation from the target frame to the model base frame
  pinocchio::FrameIndex target_frame_id;
  try {
    target_frame_id = model.getFrameId(pose_msg.header.frame_id);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Frame '%s' not found in Pinocchio model: %s", pose_msg.header.frame_id.c_str(), e.what());
    // Return original pose with failure flag
    return { target_pose, false };
  }

  // Compute forward kinematics to get the transformation
  pinocchio::forwardKinematics(model, data, q_current);
  pinocchio::updateFramePlacements(model, data);

  // Get the pose of the target frame in model base frame
  const pinocchio::SE3 target_frame_pose = data.oMf[target_frame_id];

  // Transform the target pose: pose_in_base = target_frame_pose * pose_in_target
  const pinocchio::SE3 transformed_target_pose = target_frame_pose * target_pose;

  return { transformed_target_pose, true };
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

  // Start IK worker thread
  // IK worker will be started in on_activate after Pinocchio model is built

  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianPoseController::ikWorkerMain()
{
  // Each worker maintains its own Pinocchio Data to avoid races
  pinocchio::Data local_data(pinocchio_model_);

  while (ik_worker_running_) {
    geometry_msgs::msg::PoseStamped req;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [&] { return !ik_request_queue_.empty() || !ik_worker_running_; });
      if (!ik_worker_running_) {
        break;
      }
      req = ik_request_queue_.front();
      ik_request_queue_.pop_front();
    }

    // Convert pose to SE3
    const pinocchio::SE3 target_pose = rosPoseToSE3(req.pose);

    // Snapshot q for IK
    Eigen::VectorXd q_snapshot;
    {
      std::lock_guard<std::mutex> qlock(ik_mutex_);
      if (q_snapshot_.size() == 0) {
        // no snapshot available yet; skip
        continue;
      }
      q_snapshot = q_snapshot_;
    }

    // Transform pose into model base using local_data
    auto [pose_for_ik, transform_success] = transformPoseToModelBaseFrame(
        req, target_pose, pinocchio_model_, local_data, q_snapshot, get_node()->get_logger());
    if (!transform_success) {
      continue;
    }

    Eigen::VectorXd q_out_local = Eigen::VectorXd::Zero(pinocchio_model_.nq);
    bool ik_ok = computeIK(pinocchio_model_, local_data, pose_for_ik, q_snapshot,
                           pinocchio_model_.getFrameId(params_.end_effector_frame), q_out_local);

    if (ik_ok) {
      std::lock_guard<std::mutex> lock(ik_mutex_);
      last_q_out_ = q_out_local;
      have_last_q_out_ = true;
    }
  }
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
    RCLCPP_INFO(get_node()->get_logger(), "Building Pinocchio model from URDF...");
    pinocchio::urdf::buildModelFromXML(get_robot_description(), pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model built with %zu joints", pinocchio_model_.joints.size() - 1);

    // 2. Build the collision model from urdf and srdf (only if SRDF is provided)
    if (!params_.srdf.empty()) {
      RCLCPP_INFO(get_node()->get_logger(), "Building collision geometry...");
      std::stringstream urdf_stream;
      urdf_stream << get_robot_description();
      pinocchio::urdf::buildGeom(pinocchio_model_, urdf_stream, pinocchio::COLLISION, pinocchio_geom_);

      pinocchio_geom_.addAllCollisionPairs();
      pinocchio::srdf::removeCollisionPairsFromXML(pinocchio_model_, pinocchio_geom_, params_.srdf);
      RCLCPP_INFO(get_node()->get_logger(), "Collision geometry built with %zu collision pairs",
                  pinocchio_geom_.collisionPairs.size());
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "No SRDF provided - collision checking will be disabled");
    }

    // 3. Validate that all controller joints exist in the Pinocchio model
    RCLCPP_INFO(get_node()->get_logger(), "Validating controller joints...");
    std::vector<pinocchio::JointIndex> controller_joint_indices;
    for (const auto& joint_name : params_.joints) {
      if (!pinocchio_model_.existJointName(joint_name)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      auto joint_id = pinocchio_model_.getJointId(joint_name);
      controller_joint_indices.push_back(joint_id);
      RCLCPP_INFO(get_node()->get_logger(), "Joint '%s' found with index %ld", joint_name.c_str(), joint_id);
    }

    // 4. Validate kinematic chain structure (only if more than one joint)
    if (params_.joints.size() > 1) {
      RCLCPP_INFO(get_node()->get_logger(), "Validating kinematic chain structure...");
      for (size_t i = 1; i < controller_joint_indices.size(); ++i) {
        auto current_joint_id = controller_joint_indices[i];
        auto previous_joint_id = controller_joint_indices[i - 1];

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

        RCLCPP_INFO(get_node()->get_logger(), "Chain validation for joint '%s' (id %ld) -> '%s' (id %ld): %s",
                    params_.joints[i].c_str(), current_joint_id, params_.joints[i - 1].c_str(), previous_joint_id,
                    is_valid_chain ? "VALID" : "INVALID");

        if (!is_valid_chain) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Invalid kinematic chain: Joint '%s' (index %zu) is not a descendant of joint '%s' (index %zu) "
                       "in the kinematic tree.",
                       params_.joints[i].c_str(), i, params_.joints[i - 1].c_str(), i - 1);
          RCLCPP_ERROR(get_node()->get_logger(), "Please check that the 'joints' parameter lists the joints in the "
                                                 "correct kinematic order.");
          return controller_interface::CallbackReturn::ERROR;
        }
      }
    }

    // 5. Validate end effector frame exists
    if (!pinocchio_model_.existFrame(params_.end_effector_frame)) {
      RCLCPP_ERROR(get_node()->get_logger(), "End effector frame '%s' not found in Pinocchio model.",
                   params_.end_effector_frame.c_str());

      // Debug: List all available frames
      RCLCPP_ERROR(get_node()->get_logger(), "Available frames in Pinocchio model:");
      for (size_t i = 0; i < pinocchio_model_.frames.size(); ++i) {
        RCLCPP_ERROR(get_node()->get_logger(), "  [%zu]: %s", i, pinocchio_model_.frames[i].name.c_str());
      }
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(),
                "Successfully configured controller with %zu joints and end effector frame '%s'", params_.joints.size(),
                params_.end_effector_frame.c_str());
    // Initialize q_snapshot_ now that model size is known
    {
      std::lock_guard<std::mutex> lock(ik_mutex_);
      q_snapshot_ = Eigen::VectorXd::Zero(pinocchio_model_.nq);
      have_last_q_out_ = false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during Pinocchio model setup: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Setup the pose listener
  pose_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", 10, [&](const geometry_msgs::msg::PoseStamped& msg) {
        // Write raw to RT buffer for real-time consumers
        buffer_pose_cmd_.writeFromNonRT(msg);

        // Non-RT: push to IK request queue if it's a new target (ignore duplicates)
        std::lock_guard<std::mutex> lock(queue_mutex_);
        bool duplicate = false;
        if (!ik_request_queue_.empty()) {
          const auto& back = ik_request_queue_.back();
          if (back.header.frame_id == msg.header.frame_id && back.pose.position.x == msg.pose.position.x &&
              back.pose.position.y == msg.pose.position.y && back.pose.position.z == msg.pose.position.z &&
              back.pose.orientation.w == msg.pose.orientation.w && back.pose.orientation.x == msg.pose.orientation.x &&
              back.pose.orientation.y == msg.pose.orientation.y && back.pose.orientation.z == msg.pose.orientation.z) {
            duplicate = true;
          }
        }
        if (!duplicate) {
          ik_request_queue_.push_back(msg);
          queue_cv_.notify_one();
        }
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

  // Start IK worker thread now that model and q_snapshot_ are initialized
  // Clear any stale requests before starting the worker
  {
    std::lock_guard<std::mutex> qlock(queue_mutex_);
    ik_request_queue_.clear();
  }

  if (!ik_worker_running_) {
    ik_worker_running_ = true;
    ik_worker_thread_ = std::thread(&CartesianPoseController::ikWorkerMain, this);
  }

  // Ensure the current robot pose becomes the target immediately to avoid any transient motion
  // Build full-size vectors for all robot joints (Pinocchio expects this)
  {
    const std::size_t joint_count = joint_position_state_interfaces_.size();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);

    for (std::size_t i = 0; i < joint_count; i++) {
      const std::string& joint_name = params_.joints[i];
      auto idx = pinocchio_model_.getJointId(joint_name);
      if (idx == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      q[pinocchio_model_.joints[idx].idx_q()] = joint_position_state_interfaces_.at(i).get().get_value();
      v[pinocchio_model_.joints[idx].idx_v()] = joint_velocity_state_interfaces_.at(i).get().get_value();
    }

    // Update snapshot for worker
    {
      std::lock_guard<std::mutex> lock(ik_mutex_);
      q_snapshot_ = q;
    }

    // Compute current EE pose and set it as the current target (in base frame)
    pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, q);
    pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);
    pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(params_.end_effector_frame);
    const pinocchio::SE3 current_ee_pose = pinocchio_data_.oMf[frame_id];

    // Fill PoseStamped (use empty frame_id to indicate base frame)
    geometry_msgs::msg::PoseStamped pmsg;
    pmsg.header.stamp = get_node()->now();
    pmsg.header.frame_id = "";
    const Eigen::Vector3d t = current_ee_pose.translation();
    const Eigen::Quaterniond qrot(current_ee_pose.rotation());
    pmsg.pose.position.x = t.x();
    pmsg.pose.position.y = t.y();
    pmsg.pose.position.z = t.z();
    pmsg.pose.orientation.x = qrot.x();
    pmsg.pose.orientation.y = qrot.y();
    pmsg.pose.orientation.z = qrot.z();
    pmsg.pose.orientation.w = qrot.w();

    // Write into RT buffer so update() will see the current pose as target
    buffer_pose_cmd_.writeFromNonRT(pmsg);

    // Cache IK results as the current configuration so update() won't trigger motion
    {
      std::lock_guard<std::mutex> lock(ik_mutex_);
      last_q_out_ = q;
      have_last_q_out_ = true;
    }

    // Sync command interfaces to current positions to avoid jumps when the controller starts commanding
    for (std::size_t i = 0; i < joint_count; i++) {
      double q_val = q[pinocchio_model_.joints[pinocchio_model_.getJointId(params_.joints[i])].idx_q()];
      // set the commanded position to current position
      if (!joint_position_command_interfaces_.at(i).get().set_value<double>(q_val)) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set initial command for joint '%s'",
                    params_.joints[i].c_str());
      }
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // Stop IK worker thread
  ik_worker_running_ = false;
  queue_cv_.notify_all();
  if (ik_worker_thread_.joinable()) {
    ik_worker_thread_.join();
  }

  // Clear any remaining requests after worker stopped
  {
    std::lock_guard<std::mutex> qlock(queue_mutex_);
    ik_request_queue_.clear();
  }

  active_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                  [[maybe_unused]] const rclcpp::Duration& period)
{
  using pinocchio::CollisionPair;

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

  // Make a snapshot of q for the IK worker
  {
    std::lock_guard<std::mutex> lock(ik_mutex_);
    q_snapshot_ = q;
  }

  pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(params_.end_effector_frame);

  // Use the frame id directly for IK (solve for the end-effector frame)

  const auto pose_msg = buffer_pose_cmd_.readFromRT();
  const auto target_pose = rosPoseToSE3(pose_msg->pose);

  // Transform pose to model base frame if necessary
  auto [pose_for_ik, transform_success] = transformPoseToModelBaseFrame(*pose_msg, target_pose, pinocchio_model_,
                                                                        pinocchio_data_, q, get_node()->get_logger());

  if (!transform_success) {
    return controller_interface::return_type::ERROR;
  }

  // Declare variables for IK result
  Eigen::VectorXd q_out = Eigen::VectorXd::Zero(pinocchio_model_.nq);
  bool ik_success = false;

  // Worker-only IK: do not compute IK in the hot update loop.
  // Rely on the async worker to fill `last_q_out_`. If no cached solution is available yet,
  // avoid commanding anything (safe fallback) until the worker produces a q_out.
  if (!have_last_q_out_) {
    // No IK solution available yet from the worker -> skip commanding this cycle
    return controller_interface::return_type::OK;
  }

  // Use cached q_out computed by the worker
  q_out = last_q_out_;
  ik_success = true;

  // Compute current FK for the end-effector to evaluate error (cheap compared to IK)
  pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, q);
  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);
  const pinocchio::SE3 current_ee_pose = pinocchio_data_.oMf[frame_id];

  // Compute 6D error (translation + rotation) current -> target
  const pinocchio::SE3 ee_err_se3 = current_ee_pose.actInv(pose_for_ik);
  Eigen::Matrix<double, 6, 1> ee_err6 = log6(ee_err_se3).toVector();
  const double trans_err = ee_err6.head<3>().norm();
  const double rot_err = ee_err6.tail<3>().norm();

  // Thresholds to avoid tiny commanded motions (tolerances can be tuned)
  const double TRANSLATION_TOL = 5e-4;  // 0.5 mm
  const double ROTATION_TOL = 1e-3;     // ~0.057 deg in axis-angle magnitude

  if (trans_err < TRANSLATION_TOL && rot_err < ROTATION_TOL) {
    return controller_interface::return_type::OK;
  }

  pinocchio::GeometryData geom_data(pinocchio_geom_);
  const auto collides =
      pinocchio::computeCollisions(pinocchio_model_, pinocchio_data_, pinocchio_geom_, geom_data, q_out);

  if (!collides) {
    // Compute joint-space delta and optionally clamp per-update joint changes to avoid jumps
    const double MAX_JOINT_STEP = 0.01;  // radians per update (~0.57 deg). Tune if needed.
    double max_delta_norm = 0.0;
    for (std::size_t i = 0; i < joint_count; i++) {
      const std::string& joint_name = params_.joints[i];
      auto idx = pinocchio_model_.getJointId(joint_name);
      if (idx == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
        return controller_interface::return_type::ERROR;
      }

      const double q_current_joint = q[pinocchio_model_.joints[idx].idx_q()];
      const double q_target_joint = q_out[pinocchio_model_.joints[idx].idx_q()];
      const double delta = q_target_joint - q_current_joint;
      max_delta_norm = std::max(max_delta_norm, std::abs(delta));

      // Read currently commanded position (may be last commanded value)
      double cmd_current = joint_position_command_interfaces_.at(i).get().get_value();

      // Clamp change per update to MAX_JOINT_STEP to avoid instantaneous jumps
      double applied = q_target_joint;
      if (std::abs(q_target_joint - cmd_current) > MAX_JOINT_STEP) {
        applied = cmd_current + std::copysign(MAX_JOINT_STEP, q_target_joint - cmd_current);
      }

      if (!joint_position_command_interfaces_.at(i).get().set_value<double>(applied)) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set position command for joint '%s'", joint_name.c_str());
      }
    }
  } else {
    // Print the status of all the collision pairs
    for (size_t k = 0; k < pinocchio_geom_.collisionPairs.size(); ++k) {
      const CollisionPair& cp = pinocchio_geom_.collisionPairs[k];
      const hpp::fcl::CollisionResult& cr = geom_data.collisionResults[k];

      RCLCPP_WARN(get_node()->get_logger(), "Collision pair: %s , %s - collision: %s",
                  pinocchio_geom_.geometryObjects[cp.first].name.c_str(),
                  pinocchio_geom_.geometryObjects[cp.second].name.c_str(), cr.isCollision() ? "yes" : "no");
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

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_controllers::CartesianPoseController, controller_interface::ControllerInterface)
