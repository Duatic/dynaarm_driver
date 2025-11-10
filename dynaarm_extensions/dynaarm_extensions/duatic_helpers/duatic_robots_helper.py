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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListHardwareComponents


class DuaticRobotsHelper:

    def __init__(self, node: Node):
        self.node = node

        self._joint_states = {}
        self._robot = {}
        self._robot_count = 0

        self._end_effector_keywords = (
            "finger",
            "gripper",
            "claw",
            "tool",
            "effector",
            "hand",
            "pinch",
        )

        self._joint_states_subscription = self.node.create_subscription(
            JointState, "joint_states", self._joint_sate_callback, 10
        )

        self.hardware_components_client = self.node.create_client(
            ListHardwareComponents, "controller_manager/list_hardware_components"
        )

    def _joint_sate_callback(self, msg):
        """Callback to update joint states and detect robots."""
        self._joint_states = dict(zip(msg.name, msg.position))

        if self._robot_count <= 0:
            self._robot = self.get_robots_with_components()
            self._robot_count = len(self._robot)

    def get_joint_states(self):
        """Returns a dictionary of joint names and their positions."""
        return self._joint_states

    def get_joint_value_from_states(self, joint_name):
        """Get the current position value for a specific joint from joint states."""
        if joint_name in self._joint_states:
            return self._joint_states[joint_name]
        else:
            self.node.get_logger().warning(f"Joint '{joint_name}' not found in joint states")
            return 0.0

    def get_component_names(self, filter_by_type=None):
        """Returns a list of component names, optionally filtered by type."""
        if self._robot_count <= 0:
            self.node.get_logger().error("No robots detected. Cannot get component names.")
            return []

        component_names = []
        for robot_id, components in self._robot.items():
            for component_name, component_info in components.items():
                if filter_by_type is None:
                    component_names.append(component_name)
                else:
                    found_component = component_info["type"] == filter_by_type
                    if found_component:
                        component_names.append(component_name)

        return component_names

    def get_robots_with_components(self):
        """Returns a dictionary of robots with their detected components."""
        if not self._joint_states:
            self.node.get_logger().error("No joint states received. Cannot detect robots.")
            return {}

        robots = {}

        # Analyze joint names to identify robots and their components
        for joint_name in self._joint_states.keys():
            robot_id, component_name, component_type = self._parse_joint_name(joint_name)

            # Initialize robot structure if not exists
            if robot_id not in robots:
                robots[robot_id] = {}

            # Initialize specific component if not exists
            if component_name not in robots[robot_id]:
                robots[robot_id][component_name] = {"type": component_type, "joints": []}

            # Add joint to component
            robots[robot_id][component_name]["joints"].append(joint_name)

        # Sort joints within each component for consistency
        for robot_id in robots:
            for component_name in robots[robot_id]:
                robots[robot_id][component_name]["joints"].sort()

        self.node.get_logger().debug(f"Detected robot structure: {robots}")
        return robots

    def _parse_joint_name(self, joint_name):
        """Parse a joint name to extract robot ID, component name, and component type."""
        # Handle different joint naming patterns

        # Pattern 1: arm_left/shoulder_rotation, arm_right/elbow_flexion
        if "/" in joint_name and ("arm_" in joint_name or "hand_" in joint_name):
            prefix, joint_suffix = joint_name.split("/", 1)
            if prefix.startswith("arm_"):
                return "robot_0", prefix, "arm"

        # Pattern 2: hip_pitch, hip_roll
        elif "hip" in joint_name:
            return "robot_0", "hip_0", "hip"

        # Pattern 3: joint_wheel1, joint_wheel2, joint_wheel3, joint_wheel4
        elif joint_name.startswith("joint_wheel"):
            return "robot_0", "platform_0", "platform"

        # Pattern 4: head_pan, head_tilt
        elif joint_name.startswith("head_"):
            return "robot_0", "head_0", "head"

        # Pattern 5: finger or gripper joints (e.g., zimmer_finger_left, gripper_joint)
        elif any(keyword in joint_name for keyword in self._end_effector_keywords):
            return "robot_0", "end_effector_0", "end_effector"

        # Default: treat as miscellaneous component
        return "robot_0", "", "arm"

    def get_component_count(self, component_type="arm"):
        """Get the number of components detected."""
        if self._robot_count <= 0:
            self.node.get_logger().error("No robots detected. Cannot get component count.")
            return 0

        component_count = 0
        for robot_id, components in self._robot.items():
            for component_name, component_info in components.items():
                if component_info["type"] == component_type:
                    component_count += 1

        return component_count

    def wait_for_robot(self):
        """Wait for the robot to be detected."""
        while not self._robot:
            rclpy.spin_once(self.node, timeout_sec=1.0)

        while self._robot_count <= 0:
            rclpy.spin_once(self.node, timeout_sec=1.0)

    def get_component_joint_names(
        self, robot_id="robot_0", component_type="arms", component_name=None
    ):
        """Get joints for a specific component."""
        if not self._is_robot_ready(robot_id, component_type, component_name):
            return []

        return self._robot[robot_id][component_type][component_name]["joints"]

    def get_component_joint_states(
        self, robot_id="robot_0", component_type="arms", component_name=None
    ):
        """Returns the joint_states for a component."""
        if not self._is_robot_ready(robot_id, component_type, component_name):
            return {}

        joint_states = self.get_joint_states()
        joints = self._robot[robot_id][component_type][component_name]["joints"]
        return {joint: joint_states[joint] for joint in joints if joint in joint_states}

    def _is_robot_ready(self, robot_id="robot_0", component_type="arms", component_name=None):
        """Check if the robot is ready by verifying joint states."""
        if self._robot_count <= 0:
            self.node.get_logger().error("No robots detected. Cannot get component joint states.")
            return False

        if robot_id not in self._robot:
            self.node.get_logger().error(f"Robot {robot_id} not found")
            return False

        if component_type not in self._robot[robot_id]:
            self.node.get_logger().error(f"Component type {component_type} not found in {robot_id}")
            return False

        if component_name not in self._robot[robot_id][component_type]:
            self.node.get_logger().error(f"Component {component_name} not found")
            return False

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
                self.node.get_logger().debug(f"Mock hardware detected: {component.plugin_name}")
                is_simulation = True
            elif "fake" in plugin_name:
                self.node.get_logger().debug(f"Fake hardware detected: {component.plugin_name}")
                is_simulation = True
            elif "gazebo" in plugin_name:
                self.node.get_logger().debug(f"Gazebo hardware detected: {component.plugin_name}")
                is_simulation = True
            elif any(real_hw in plugin_name for real_hw in ["real"]):
                self.node.get_logger().debug(f"Real hardware detected: {component.plugin_name}")
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
