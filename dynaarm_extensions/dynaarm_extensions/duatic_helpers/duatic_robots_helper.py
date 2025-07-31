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

import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class DuaticRobotsHelper:

    def __init__(self, node: Node):
        self.node = node

        self._robot_count = 0
        self._joint_states = {}

        self._joint_states_subscription = self.node.create_subscription(
            JointState, "/joint_states", self._joint_sate_callback, 10
        )

    def _joint_sate_callback(self, msg):
        """Callback to update joint states and detect robots."""
        self._joint_states = dict(zip(msg.name, msg.position))

        if self._robot_count <= 0:
            self._check_robot_amount()

    def get_joint_states(self):
        """Returns a dictionary of joint names and their positions."""
        return self._joint_states

    def get_robot_count(self):
        """Get the number of robots by checking joint names in /joint_states."""
        while self._robot_count <= 0:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        return self._robot_count

    def _check_robot_amount(self):
        """Robustly detect the number of robots by analyzing joint name prefixes from /joint_states."""
        self.node.get_logger().info("Waiting for /joint_states to detect robots...")

        if not self._joint_states:
            self.node.get_logger().error("No joint states received. Cannot detect robots.")
            rclpy.shutdown()
            sys.exit(1)

        # Extract unique robot prefixes based on joint naming patterns
        prefixes = set()
        for joint_name in self._joint_states.keys():
            if "/" in joint_name:
                # Multi-arm case: 'arm_right/shoulder_rotation' -> 'arm_right'
                prefix = joint_name.split("/")[0]
                prefixes.add(prefix)
            else:
                # Single arm case: 'shoulder_rotation' -> no prefix (None)
                prefixes.add("None")

        count = len(prefixes)

        # Log detected prefixes for debugging
        prefix_list = list(prefixes)
        self.node.get_logger().info(f"Detected {count} robot(s) with prefixes: {prefix_list}")

        if count > 2:
            self.node.get_logger().error(
                "More than 2 robots detected by joint name prefix. Only up to two are supported."
            )
            rclpy.shutdown()
            sys.exit(1)

        self._robot_count = count

    def check_simulation_mode(self):
        """Detect if we're running in simulation or real hardware mode."""
        try:
            node_names = self.node.get_node_names()
            if "gz_ros_control" in node_names:
                return True
        except Exception as e:
            self.node.get_logger().debug(f"Could not check for gz_ros_control: {e}")

        return False
