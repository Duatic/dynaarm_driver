# Copyright 2025 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import xacro
import tempfile

import numpy as np
import pinocchio as pin
from dynaarm_extensions.duatic_helpers.duatic_param_helper import DuaticParamHelper

from geometry_msgs.msg import PoseStamped
from ament_index_python import get_package_share_directory


class DuaticPinocchioHelper:
    """A simple class to retrieve the robot URDF from the parameter server."""

    def __init__(self, node, robot_part_name="", robot_type="DynaArm", joint_margins=0.2):
        self.node = node
        self.robot_type = robot_type
        self.robot_part_name = robot_part_name
        self.param_helper = DuaticParamHelper(self.node)

        self.model = self.get_pin_model_data()
        self.data = self.model.createData()

        # Filter joint names to only include the specified robot part (if any)
        self.joint_names = self._get_part_joint_names()

        # Get joint limits for this specific robot part
        self._set_part_joint_limits(joint_margins)

        self.lower = self.model.lowerPositionLimit + joint_margins
        self.upper = self.model.upperPositionLimit - joint_margins

    def _get_part_joint_names(self):
        """Get joint names that belong to the specified robot part."""
        if not self.robot_part_name:
            # Return all joints if no specific part is specified
            return [name for name in self.model.names[1:]]
        
        part_joint_names = []
        all_joint_names = [name for name in self.model.names[1:]]  # Skip universe joint
        
        for name in all_joint_names:
            if self.robot_part_name in name:
                part_joint_names.append(name)
        
        self.node.get_logger().info(f"Found {len(part_joint_names)} joints for {self.robot_part_name}: {part_joint_names}")
        return part_joint_names

    def _set_part_joint_limits(self, joint_margins):
        """Set joint limits for the specified robot part only."""
        if not self.robot_part_name:
            # Use all joint limits if no specific part is specified
            self.lower = self.model.lowerPositionLimit + joint_margins
            self.upper = self.model.upperPositionLimit - joint_margins
            return
        
        # Get indices of part joints in the full model
        part_joint_indices = []
        all_joint_names = [name for name in self.model.names[1:]]
        
        for joint_name in self.joint_names:
            if joint_name in all_joint_names:
                part_joint_indices.append(all_joint_names.index(joint_name))
        
        # Extract limits for part joints only
        self.lower = self.model.lowerPositionLimit[part_joint_indices] + joint_margins
        self.upper = self.model.upperPositionLimit[part_joint_indices] - joint_margins

    def get_fk(self, current_joint_values, frame):
        """Compute forward kinematics for the specified arm."""
        # Create full joint configuration (all joints in model)
        q_full = np.zeros(self.model.nq)
        
        # If dict, use joint names; if list/array, assume correct order for this arm
        if isinstance(current_joint_values, dict):
            arm_joint_values = np.array(
                [current_joint_values[name] for name in self.joint_names], dtype=np.float64
            )
        else:
            arm_joint_values = np.array(current_joint_values, dtype=np.float64)
        
        # Map arm joint values to full configuration
        all_joint_names = [name for name in self.model.names[1:]]
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in all_joint_names:
                full_index = all_joint_names.index(joint_name)
                q_full[full_index] = arm_joint_values[i]

        # Compute FK
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        frame_id = self.model.getFrameId(frame)
        return self.data.oMf[frame_id]

    def get_fk_as_pose_stamped(self, current_joint_values, frame, offsets=[0.0, 0.0, 0.085]):

        current_pose = self.get_fk(current_joint_values, frame)

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = current_pose.translation[0] - offsets[0]
        pose_stamped.pose.position.y = current_pose.translation[1] - offsets[1]
        pose_stamped.pose.position.z = current_pose.translation[2] - offsets[2]

        quat = pin.Quaternion(current_pose.rotation)
        pose_stamped.pose.orientation.x = quat.x
        pose_stamped.pose.orientation.y = quat.y
        pose_stamped.pose.orientation.z = quat.z
        pose_stamped.pose.orientation.w = quat.w

        return pose_stamped

    def solve_ik(self, q_arm, target_SE3, frame, iterations=100, threshold=0.0001, joint_weights=None):
        """Solve IK for the specified arm only."""
        error = np.inf

        if joint_weights is None:
            joint_weights = np.ones_like(q_arm)
        W = np.diag(joint_weights)

        frame_id = self.model.getFrameId(frame)

        for i in range(iterations):
            error = self.get_pose_error(q_arm, target_SE3)
            if np.linalg.norm(error) < threshold:
                break

            # Create full configuration for Jacobian computation
            q_full = np.zeros(self.model.nq)
            all_joint_names = [name for name in self.model.names[1:]]
            for j, joint_name in enumerate(self.joint_names):
                if joint_name in all_joint_names:
                    full_index = all_joint_names.index(joint_name)
                    q_full[full_index] = q_arm[j]

            # Compute Jacobian for full model
            J_full = pin.computeFrameJacobian(self.model, self.data, q_full, frame_id, pin.LOCAL)
            
            # Extract Jacobian columns for arm joints only
            arm_joint_indices = []
            for joint_name in self.joint_names:
                if joint_name in all_joint_names:
                    arm_joint_indices.append(all_joint_names.index(joint_name))
            
            J = J_full[:, arm_joint_indices]
            J_w = J @ W

            dq = -0.1 * W @ np.linalg.pinv(J_w) @ error

            q_arm += dq
            q_arm = np.clip(q_arm, self.lower, self.upper)

        return q_arm, error

    def get_pose_error(self, q, target_SE3, frame):
        current_SE3 = self.get_fk(q, frame)
        error = pin.log(target_SE3.inverse() * current_SE3).vector
        # Scale rotation part (last 3 elements) to balance translation/rotation error
        rot_scale = 0.2  # Try 0.1–0.2 for typical arms
        error[3:] *= rot_scale
        return error

    def convert_pose_stamped_to_se3(self, pose_stamped, offsets=[0.0, 0.0, 0.085]):
        """Convert a PoseStamped message to a Pinocchio SE3 object."""
        pos = pose_stamped.pose.position
        ori = pose_stamped.pose.orientation
        translation = np.array([pos.x + offsets[0], pos.y + offsets[1], pos.z + offsets[2]])
        quat = np.array([ori.w, ori.x, ori.y, ori.z])
        return pin.SE3(pin.Quaternion(*quat).matrix(), translation)

    def get_pin_model_data(self):

        # Try to get URDF from parameter server first (preferred)
        urdf_xml = self.get_robot_urdf_from_param()
        
        if urdf_xml:
            # Use URDF from parameter server
            urdf_file_path = self._write_urdf_to_temp_file(urdf_xml)
        else:
            # Fallback to hardcoded path if parameter server fails
            self.node.get_logger().warn("URDF not found on parameter server, using fallback URDF")
            urdf_file_path = self.get_dynaarm_urdf()

        # Load model with Pinocchio
        self.model = pin.buildModelFromUrdf(urdf_file_path)
        self.node.get_logger().info(
            f"Pinocchio model loaded with {len(self.model.joints)} joints and {len(self.model.frames)} frames."
        )
        return self.model    

    def _write_urdf_to_temp_file(self, urdf_xml):
        """Write URDF XML string to a temporary file."""
        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
            urdf_file.write(urdf_xml.encode())
            return urdf_file.name

    def get_dynaarm_urdf(self):

        package_share_desc_name = "dynaarm_single_example_description"
        urdf_name = "dynaarm_single_example.urdf.xacro"

        if self.robot_type == "Alpha":
            # Use the default URDF for the whole robot
            package_share_desc_name = "alpha_example_description"
            urdf_name = "alpha_example.urdf.xacro"

        # Load the robot description
        pkg_share_description = get_package_share_directory(package_share_desc_name)
        xacro_path = os.path.join(pkg_share_description, "urdf", urdf_name)
        urdf_xml = xacro.process_file(xacro_path).toxml()

        # Write URDF to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
            urdf_file.write(urdf_xml.encode())
            urdf_file_path = urdf_file.name
            return urdf_file_path

    def get_robot_urdf_from_param(self):
        """Retrieve the robot URDF from the parameter server."""
        try:
            urdf_values = self.param_helper.get_param_values(
                "robot_state_publisher", "robot_description"
            )
            if urdf_values and urdf_values[0].string_value:
                self.node.get_logger().info("Successfully loaded URDF from parameter server")
                return urdf_values[0].string_value
            else:
                self.node.get_logger().warn("URDF parameter exists but is empty")
                return None
        except Exception as e:
            self.node.get_logger().warn(f"Failed to get URDF from parameter server: {e}")
            return None
