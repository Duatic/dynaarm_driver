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

    def __init__(self, node, ee_frame_name="flange", joint_margins=0.1):        
        self.node = node
        self.param_helper = DuaticParamHelper(self.node)

        self.model = self.get_pin_model_data()
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(ee_frame_name)
        self.ee_frame_name = ee_frame_name
        self.joint_names = [name for name in self.model.names[1:]]       
        
        self.lower = self.model.lowerPositionLimit + joint_margins
        self.upper = self.model.upperPositionLimit - joint_margins

    def get_fk(self, current_joint_values):
        # Ensure joint order matches the model
        
        # If dict, use joint names; if list/array, assume correct order
        if isinstance(current_joint_values, dict):
            q = np.array([current_joint_values[name] for name in self.joint_names], dtype=np.float64)
        else:
            q = np.array(current_joint_values, dtype=np.float64)

        # Compute FK
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        return self.data.oMf[self.frame_id]        
    
    def get_fk_as_pose_stamped(self, current_joint_values):

        current_pose = self.get_fk(current_joint_values)

        pose_stamped = PoseStamped()     
        pose_stamped.pose.position.x = current_pose.translation[0] 
        pose_stamped.pose.position.y = current_pose.translation[1]
        pose_stamped.pose.position.z = current_pose.translation[2] - 0.085
        
        quat = pin.Quaternion(current_pose.rotation)
        pose_stamped.pose.orientation.x = quat.x
        pose_stamped.pose.orientation.y = quat.y
        pose_stamped.pose.orientation.z = quat.z
        pose_stamped.pose.orientation.w = quat.w

        return pose_stamped

    def solve_ik(self, q, target_SE3, iterations=100, threshold=0.0001, joint_weights=None):
        error = np.inf

        if joint_weights is None:
            joint_weights = np.ones_like(q)
        W = np.diag(joint_weights)

        for i in range(iterations):
            error = self.get_pose_error(q, target_SE3)
            if np.linalg.norm(error) < threshold:
                break

            J = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id, pin.LOCAL)
            J_w = J @ W

            dq = -0.1 * W @ np.linalg.pinv(J_w) @ error

            q += dq
            q = np.clip(q, self.lower, self.upper)

        return q, error

    def get_pose_error(self, q, target_SE3):
        current_SE3 = self.get_fk(q)
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

        urdf_file_path = self.get_dynaarm_urdf()

        # Load model with Pinocchio
        self.model = pin.buildModelFromUrdf(urdf_file_path)
        print("model name: " + self.model.name)
        return self.model

    def get_dynaarm_urdf(self):

        # Load the robot description
        pkg_share_description = get_package_share_directory(
            "dynaarm_single_example_description"
        )
        xacro_path = os.path.join(
            pkg_share_description, "urdf/dynaarm_single_example.urdf.xacro"
        )
        urdf_xml = xacro.process_file(xacro_path).toxml()

        # Write URDF to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
            urdf_file.write(urdf_xml.encode())
            urdf_file_path = urdf_file.name
            return urdf_file_path

    def get_robot_urdf_from_param(self):
        """Retrieve the robot URDF from the parameter server."""
        urdf_values = self.param_helper.get_param_values(
            "robot_state_publisher", "robot_description"
        )
        if urdf_values and urdf_values[0].string_value:
            return urdf_values[0].string_value
        else:
            self.node.get_logger().error("URDF not found on parameter server.")
            return None
