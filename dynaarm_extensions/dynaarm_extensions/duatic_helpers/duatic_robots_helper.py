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
from controller_manager_msgs.srv import ListHardwareComponents


class DuaticRobotsHelper:

    def __init__(self, node: Node):
        self.node = node

        self._robot_count = 0
        self._joint_states = {}

        self._joint_states_subscription = self.node.create_subscription(
            JointState, "/joint_states", self._joint_sate_callback, 10
        )

        self.hardware_components_client = self.node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
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
        is_simulation = False

        if not self.hardware_components_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().debug("Hardware components service not available")
            return is_simulation

        # Call the service
        request = ListHardwareComponents.Request()
        future = self.hardware_components_client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.result() is None:
            self.node.get_logger().debug("Hardware components service call failed")
            return 0

        response = future.result()
        for component in response.component:
            plugin_name = component.plugin_name.lower()

            # Check plugin name for mock hardware indicators
            if "mock" in plugin_name:
                self.node.get_logger().info(f"Mock hardware detected: {component.plugin_name}")
                is_simulation = True
            elif "fake" in plugin_name:
                self.node.get_logger().info(f"Fake hardware detected: {component.plugin_name}")
                is_simulation = True
            elif "gazebo" in plugin_name:
                self.node.get_logger().info(f"Gazebo hardware detected: {component.plugin_name}")
                is_simulation = True
            elif any(real_hw in plugin_name for real_hw in ["real"]):
                self.node.get_logger().info(f"Real hardware detected: {component.plugin_name}")
                is_simulation = False

        return is_simulation

    def get_dt(self):
        self.is_simulation = self.check_simulation_mode()

        if self.is_simulation:
            self.dt = 0.05
            self.node.get_logger().info("Using simulation timing: dt=0.05s (20Hz)")
        else:
            self.dt = 0.001
            self.node.get_logger().info("Using real hardware timing: dt=0.001s (1000Hz)")

        return self.dt  # Return the dt value

    def get_joint_states_per_arm(self):
        """Returns joint states organized per arm."""
        joint_states = self.get_joint_states()
        arms_count = self.get_robot_count()

        if arms_count <= 1:
            return [joint_states]  # Always return a list, even for single arm

        # Multi-arm: split joint_states by arm prefix
        arm_states = []
        joint_names = list(joint_states.keys())

        # Group joints by their prefix (arm_right, arm_left, etc.)
        arms_data = {}
        for joint_name in joint_names:
            if "/" in joint_name:
                prefix = joint_name.split("/")[0]  # e.g., 'arm_right'
                if prefix not in arms_data:
                    arms_data[prefix] = {}
                arms_data[prefix][joint_name] = joint_states[joint_name]
            else:
                # Single arm case - should not happen with arms_count > 1
                self.node.get_logger().warn(
                    f"Multi-arm setup but found joint without prefix: {joint_name}"
                )

        # Convert to list format (maintain order: arm_right first, then arm_left)
        arm_prefixes = sorted(arms_data.keys())  # Ensures consistent ordering
        for prefix in arm_prefixes:
            arm_states.append(arms_data[prefix])

        return arm_states

    def extract_joint_values_for_arm(self, arm_index, joint_names):
        """Extract joint values for a specific arm by index."""
        arms_joint_states = self.get_joint_states_per_arm()

        if arm_index >= len(arms_joint_states):
            self.node.get_logger().error(
                f"Arm index {arm_index} out of range. Only {len(arms_joint_states)} arms available."
            )
            return []

        arm_joint_dict = arms_joint_states[arm_index]

        # Check if all required joint names exist in this arm's data
        if all(name in arm_joint_dict for name in joint_names):
            return [arm_joint_dict[name] for name in joint_names]

        self.node.get_logger().error(
            f"Not all joint names found for arm {arm_index}: {joint_names}"
        )
        return []
