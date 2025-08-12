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


import numpy as np
from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dynaarm_extensions.duatic_helpers.duatic_pinocchio_helper import DuaticPinocchioHelper
from dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper


class PoseControllerNode(Node):
    def __init__(self):
        super().__init__("dynaarm_pose_controller_node")

        self.declare_parameter("arm_name", "")
        self.arm_name = self.get_parameter("arm_name").get_parameter_value().string_value
        self.arm_name_for_topics = f"_{self.arm_name}" if self.arm_name else ""

        self.latest_pose = None

        if self.arm_name:    
            self.frame = f"{self.arm_name}/flange"
            self.base_frame = "tbase"
            self.pin_helper = DuaticPinocchioHelper(self, robot_type="Alpha")
        else:
            self.frame = "flange"
            self.base_frame = "world"
            self.pin_helper = DuaticPinocchioHelper(self)

        self.robot_helper = DuaticRobotsHelper(self)
        self.dt = self.robot_helper.get_dt()
        self.active = False

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f"/dynaarm_pose_controller{self.arm_name_for_topics}/target_frame",
            self.pose_callback,
            10,
        )

        # Publishers
        self.jtc_pub = self.create_publisher(
            JointTrajectory,
            f"/joint_trajectory_controller{self.arm_name_for_topics}/joint_trajectory",
            10,
        )

        # Service to activate/deactivate the controller
        self.srv = self.create_service(
            SetBool,
            f"dynaarm_pose_controller{self.arm_name_for_topics}/activate",
            self.handle_activate_service,
        )

        # Start periodic control loop
        self.max_distance = 0.2        
        self.max_joint_speed = 2.0  # rad/s
        self.control_timer = self.create_timer(self.dt, self.control_loop)

    def handle_activate_service(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"Pose controller {'activated' if self.active else 'deactivated'}"        
        self.latest_pose = None

        return response

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def control_loop(self):
        if not self.active:
            return

        if self.latest_pose is None:
            return

        current_joint_values = self.robot_helper.get_joint_states()
        if len(current_joint_values) <= 0:
            self.get_logger().warn("No joint states received yet.", throttle_duration_sec=1.0)
            return

        # Rest of the method remains the same...
        target_SE3 = self.pin_helper.convert_pose_stamped_to_se3(self.latest_pose)
        error = self.pin_helper.get_pose_error(current_joint_values, target_SE3, self.frame, self.base_frame)

        if not self.is_move_safe(error):
            # Extract only the arm-specific joints from current state
            arm_joint_positions, arm_joint_names = self.extract_arm_joints(current_joint_values)
            self.send_joint_trajectory(arm_joint_names, arm_joint_positions)
            return

        # Convert all joint states to full configuration array
        q, error = self.pin_helper.solve_ik(current_joint_values, target_SE3, self.frame, self.base_frame)

        if np.linalg.norm(error) >= 0.01:
            self.get_logger().warn("Can't find any valid IK solution", throttle_duration_sec=2.0)
            return

        # Extract only the arm-specific joints from the IK solution
        arm_joint_positions, arm_joint_names = self.extract_arm_joints(q)
        self.send_joint_trajectory(arm_joint_names, arm_joint_positions)

    def extract_arm_joints(self, joint_data):
        """Extract joint positions for this specific arm from either dict or array."""
        # Handle dictionary input (from current_joint_values)
        if isinstance(joint_data, dict):
            arm_joint_positions = []
            arm_joint_names = []

            # If no specific arm is configured, return all joints
            if not self.arm_name:
                return list(joint_data.values()), list(joint_data.keys())

            # Filter joints that belong to this specific arm
            for joint_name, joint_value in joint_data.items():
                if joint_name.startswith(f"{self.arm_name}/"):
                    arm_joint_positions.append(joint_value)
                    arm_joint_names.append(joint_name)

            return arm_joint_positions, arm_joint_names

        # Handle array/list input (from IK solution q)
        else:
            # If no specific arm is configured, return all joints
            if not self.arm_name:
                all_joint_names = self.pin_helper.get_all_joint_names()
                return list(joint_data), all_joint_names

            # Use the new helper function to extract arm-specific joints
            arm_joint_positions, arm_joint_names = self.pin_helper.get_joint_values_by_arm_prefix(
                joint_data, self.arm_name
            )

            return arm_joint_positions, arm_joint_names

    def is_move_safe(self, error):
        """Determines whether a move to the target pose is considered 'safe'."""
        normed_error = np.linalg.norm(error)
        if normed_error > self.max_distance:
            self.get_logger().warn(
                f"Target pose too far away ({normed_error} m > {self.max_distance:.2f} m). Ignoring command.",
                throttle_duration_sec=5.0,
            )
            return False

        return True

    def send_joint_trajectory(self, joint_names, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions

        # Get current joint positions
        current_joint_values = self.robot_helper.get_joint_states()
        current_positions = [current_joint_values[name] for name in joint_names]

        # Compute required time for each joint
        diffs = [abs(p - c) for p, c in zip(joint_positions, current_positions)]
        times = [d / self.max_joint_speed if self.max_joint_speed > 0 else 0.2 for d in diffs]
        min_time = 0.2  # minimum time for safety/smoothness
        duration = max(max(times), min_time)

        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        traj_msg.points.append(point)
        self.jtc_pub.publish(traj_msg)


def main(args=None):

    rclpy.init(args=args)
    node = PoseControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
