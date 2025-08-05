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

    def __init__(self, node, robot_type="DynaArm", joint_margins=0.2):
        self.node = node
        self.robot_type = robot_type
        self.param_helper = DuaticParamHelper(self.node)

        self.model = self.get_pin_model_data()
        self.data = self.model.createData()

        self.lower = self.model.lowerPositionLimit + joint_margins
        self.upper = self.model.upperPositionLimit - joint_margins

    def _convert_joint_values_to_array(self, current_joint_values):
        """Convert joint values (dict or array) to a proper numpy array for Pinocchio."""
        if isinstance(current_joint_values, dict):
            # Create full joint configuration array from dictionary
            q = np.zeros(self.model.nq)
            all_joint_names = [name for name in self.model.names[1:]]  # Skip universe joint

            for i, joint_name in enumerate(all_joint_names):
                if joint_name in current_joint_values:
                    q[i] = current_joint_values[joint_name]
                else:
                    # Use 0.0 for missing joints
                    q[i] = 0.0
        else:
            # Convert list/array to numpy array
            q = np.array(current_joint_values, dtype=np.float64)

            # If we don't have enough joints, pad with zeros
            if len(q) < self.model.nq:
                q_full = np.zeros(self.model.nq)
                q_full[: len(q)] = q
                q = q_full

        return q

    def get_fk(self, current_joint_values, frame):
        """Compute forward kinematics for the specified arm."""
        # Convert input to proper numpy array
        q = self._convert_joint_values_to_array(current_joint_values)

        # Compute FK
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        frame_id = self.model.getFrameId(frame)
        return self.data.oMf[frame_id]

    def get_fk_as_pose_stamped(self, current_joint_values, frame, offsets=[0.0, 0.0, 0.0]):

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

    def solve_ik(self, current_joint_values, target_SE3, frame, iterations=100, threshold=0.0001):
        """Solve IK for the specified arm, but work with full joint configuration."""
        error = np.inf

        # Convert input to proper numpy array (always full configuration)
        q = self._convert_joint_values_to_array(current_joint_values)

        joint_weights = np.ones_like(q)
        W = np.diag(joint_weights)

        frame_id = self.model.getFrameId(frame)

        for i in range(iterations):

            # Compute error using full configuration
            error = self.get_pose_error(q, target_SE3, frame)
            if np.linalg.norm(error) < threshold:
                break

            # Compute Jacobian for full model
            J = pin.computeFrameJacobian(self.model, self.data, q, frame_id, pin.LOCAL)

            # Extract Jacobian columns for arm joints only
            J_w = J @ W
            dq = -0.1 * W @ np.linalg.pinv(J_w) @ error

            q += dq
            # Use only the arm-specific limits for clipping
            q = np.clip(q, self.lower, self.upper)

        return q, error

    def get_pose_error(self, q, target_SE3, frame):
        current_SE3 = self.get_fk(q, frame)
        error = pin.log(target_SE3.inverse() * current_SE3).vector
        # Scale rotation part (last 3 elements) to balance translation/rotation error
        rot_scale = 0.2  # Try 0.1–0.2 for typical arms
        error[3:] *= rot_scale
        return error

    def convert_pose_stamped_to_se3(self, pose_stamped, offsets=[0.0, 0.0, 0.0]):
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

    def get_joint_values_by_names(self, q, joint_names):
        """Extract specific joint values from q array by joint names."""
        joint_values = []
        all_joint_names = [name for name in self.model.names[1:]]  # Skip universe joint

        for joint_name in joint_names:
            try:
                joint_index = all_joint_names.index(joint_name)
                joint_values.append(q[joint_index])
            except ValueError:
                self.node.get_logger().warn(f"Joint '{joint_name}' not found in model")
                joint_values.append(0.0)  # Default value for missing joints

        return joint_values

    def get_joint_values_by_arm_prefix(self, q, arm_prefix):
        """Extract joint values from q array for joints starting with arm_prefix."""
        joint_values = []
        joint_names = []
        all_joint_names = [name for name in self.model.names[1:]]  # Skip universe joint

        for i, joint_name in enumerate(all_joint_names):
            if joint_name.startswith(f"{arm_prefix}/"):
                joint_values.append(q[i])
                joint_names.append(joint_name)

        return joint_values, joint_names

    def get_all_joint_names(self):
        """Get all joint names from the model (excluding universe joint)."""
        return [name for name in self.model.names[1:]]
