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
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import re
from rclpy.parameter_client import AsyncParameterClient
import time
from controller_manager_msgs.srv import ListControllers, SwitchController
from dynaarm_extensions.dynaarm_controller_manager import DynAarmControllerManager
from ament_index_python.packages import get_package_share_directory
import os
import yaml


class MoveToPredefinedPositionNode(Node):

    MIRRORED_BASES = {"shoulder_rotation", "forearm_rotation", "wrist_rotation"}

    def __init__(self):
        super().__init__('motion_to_predefined_position_node')

        config_path = os.path.join(
            get_package_share_directory("dynaarm_extensions"),
            "config",
            "controllers.yaml",
        )

        with open(config_path) as file:
            config = yaml.safe_load(file)

        self.controller_manager = DynAarmControllerManager(self, config["controllers"])

        self.declare_parameter("robot_configuration", "dynaarm_dual")  # Default configuration
        self.robot_configuration = self.get_parameter("robot_configuration").value

        if self.robot_configuration == "dynaarm" or self.robot_configuration == "dynaarm_flip": 
            num_arms = 1
        elif self.robot_configuration == "alpha" or self.robot_configuration == "dynaarm_dual":
            num_arms = 2

        self.list_controllers_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.sleep_position_dynaarm = [
            0.0,
            -1.720,
            3.14159,
            0.0,
            1.0,
            0.0,
        ]  # Sleep position for DynAarm
        self.home_position_dynaarm = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # Home position for DynAarm
        self.home_position_alpha = [
            1.5708,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ] # Home position for Alpha
        self.sleep_position_alpha = [
            1.5708,
            0.680678,
            0.0,
            0.0,
            0.0,
            0.0,
        ] # Sleep position for Alpha

        self.home = False  # Initialize home position flag
        self.sleep = False  # Initialize sleep position flag
        self.step_size_flexion_joints = 0.01 # Step size for flexion joints
        self.step_size_rotation_joints = 0.01 # Step size for rotation joints
        self.flexion_joints_indices = [1, 2, 4] # Indices of flexion joints
        self.rotation_joints_indices = [0, 3, 5] # Indices of rotation joints
        self.tolerance = 0.01 # Tolerance for joint angle comparison
        self.dt = 0.05 # Control loop period in seconds
        self.controller_active = False  # Flag to check if the controller is active

        # Subscriptions
        self.home_subscriber = self.create_subscription(
            Bool,
            'move_home',
            self.move_home_callback,
            10
        )
        self.sleep_subscriber = self.create_subscription(
            Bool,
            'move_sleep',
            self.move_sleep_callback,
            10
        )
        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )

        self.joint_trajectory_publishers = {} # Dictionary to hold publishers for joint trajectory topics
        self.topic_to_joint_names = {} # Dictionary to map topics to joint names
        self.topic_to_commanded_positions = {} # Dictionary to hold commanded positions for each topic
        self.prefix_to_joints = {} # Dictionary to map prefixes to joint names
        found_topics = {} # Dictionary to hold found topics
        self.previous_controller = {}
        topic_prefix = "/joint_trajectory_controller"

        # Find all joint trajectory topics matching the prefix
        while len(found_topics) != num_arms:
            found_topics = self.get_topic_names_and_types_test(f"{topic_prefix}*/joint_trajectory")
            time.sleep(0.1)  # Wait for topics to be discovered

        # Discover all topics and joint names, extract prefix
        for topic, types in found_topics:  #
            self.joint_trajectory_publishers[topic] = self.create_publisher(
                JointTrajectory, topic, 10
            )
            # Extract prefix from topic name
            # e.g. /joint_trajectory_controller_arm_1/joint_trajectory -> arm_1
            joint_names = self.get_param_values(topic.split("/")[1], "joints")
            if joint_names:
                self.topic_to_joint_names[topic] = joint_names
                self.topic_to_commanded_positions[topic] = [0.0] * len(joint_names)
            else:
                print("Parameter not found or empty for topic", topic)

        self.all_controllers = self.controller_manager.get_all_controllers()
        self.create_timer(self.dt, self.control_loop)

    def move_home_callback(self, msg):
        self.home = msg.data
        self.sleep = False  # Reset sleep flag when moving to home

    def move_sleep_callback(self, msg):
        self.sleep = msg.data
        self.home = False  # Reset home flag when moving to sleep

    def joint_state_callback(self, msg: JointState):
        """Update stored joint states and set initial positions."""
        self.joint_states = dict(zip(msg.name, msg.position))

    # Control loop that checks the home and sleep flags and moves the robot accordingly
    def control_loop(self):
        if not self.controller_manager.is_freeze_active:
            if self.home:
                if not self.controller_active:
                    self.switch_to_joint_trajectory_controllers()
                    self.controller_active = True
                if self.robot_configuration == "dynaarm" or self.robot_configuration == "dynaarm_dual":
                    self.move_home_dynaarm()
                elif self.robot_configuration == "alpha":
                    self.move_home_alpha()
                elif self.robot_configuration == "dynaarm_flip":
                    self.move_home_dynaarm()
            if self.sleep:
                if not self.controller_active:
                    self.switch_to_joint_trajectory_controllers()
                    self.controller_active = True
                if self.robot_configuration == "dynaarm" or self.robot_configuration == "dynaarm_dual":
                    self.move_sleep_dynaarm()
                if self.robot_configuration == "alpha":
                    self.move_sleep_alpha()
                if self.robot_configuration == "dynaarm_flip":
                    self.move_home_dynaarm()
            if not self.home and not self.sleep:
                if self.controller_active:
                    self.switch_to_previous_controllers()
                    self.controller_active = False
                self.previous_controller = next(iter(self.controller_manager.active_controllers), None)

    # Move to home for DynAarm Configuration
    def move_home_dynaarm(self):
        for topic, joint_names in self.topic_to_joint_names.items():
            commanded_positions = self.move_to_position(
                    joint_names, self.home_position_dynaarm.copy()
                ) 
            self.topic_to_commanded_positions[topic] = commanded_positions

        for topic, publisher in self.joint_trajectory_publishers.items():
            self.publish_joint_trajectory(
                self.topic_to_commanded_positions[topic],
                publisher=publisher,
                joint_names=self.topic_to_joint_names[topic],
            )

    # Move to sleep for DynAarm Configuration
    def move_sleep_dynaarm(self):
        for topic, joint_names in self.topic_to_joint_names.items():
            current_joint_values = self.extract_joint_values(self.get_joint_states(self.arms_count), joint_names)
            if self.joints_at_pose(current_joint_values.copy(), self.rotation_joints_indices, self.home_position_dynaarm.copy()): # Check if rotation joints are at home position
                commanded_positions = self.move_to_position(
                    joint_names, self.sleep_position_dynaarm.copy()
                )
            else:
                commanded_positions = self.move_to_position(
                    joint_names, self.home_position_dynaarm.copy()
                )
            self.topic_to_commanded_positions[topic] = commanded_positions

        for topic, publisher in self.joint_trajectory_publishers.items():
            self.publish_joint_trajectory(
                self.topic_to_commanded_positions[topic],
                publisher=publisher,
                joint_names=self.topic_to_joint_names[topic],
            )

    # Move to home for Alpha Configuration
    def move_home_alpha(self):
        mirror_arm = False
        for topic, joint_names in self.topic_to_joint_names.items():
            if mirror_arm: # Mirror the position for the second arm
                target_home_position = self.mirror_position(
                        joint_names, self.home_position_alpha.copy()
                    )
            else:
                target_home_position = self.home_position_alpha.copy()
            commanded_positions = self.move_to_position(
                joint_names, target_home_position
            ) 
            self.topic_to_commanded_positions[topic] = commanded_positions
            mirror_arm = True

        for topic, publisher in self.joint_trajectory_publishers.items():
            self.publish_joint_trajectory(
                self.topic_to_commanded_positions[topic],
                publisher=publisher,
                joint_names=self.topic_to_joint_names[topic],
            )

    # Move to sleep for Alpha Configuration
    def move_sleep_alpha(self):
        mirror_arm = False
        for topic, joint_names in self.topic_to_joint_names.items():
            if not self.joint_angles_equal(self.extract_joint_values(self.get_joint_states(self.arms_count), joint_names), 
                                           self.mirror_position(joint_names, self.home_position_alpha.copy()) 
                                           if mirror_arm else self.home_position_alpha.copy()): # Check if current position is not equal to home position
                target_position = self.home_position_alpha.copy()
            else: 
                target_position = self.sleep_position_alpha.copy()

            if mirror_arm: # Mirror the position for the second arm
                target_position = self.mirror_position(
                        joint_names, target_position
                    )
                
            commanded_positions = self.move_to_position(
                joint_names, target_position
            ) 
            self.topic_to_commanded_positions[topic] = commanded_positions
            mirror_arm = not mirror_arm  # Toggle mirroring for next arm

        for topic, publisher in self.joint_trajectory_publishers.items():
            self.publish_joint_trajectory(
                self.topic_to_commanded_positions[topic],
                publisher=publisher,
                joint_names=self.topic_to_joint_names[topic],
            )

    # Move to a specific position for the given joint names
    def move_to_position(self, joint_names, target_position):
        current_joint_values = self.extract_joint_values(self.get_joint_states(self.arms_count), joint_names)
        if self.joints_at_pose(current_joint_values.copy(), self.flexion_joints_indices, target_position): # Check if flexion joints are at target position
            next_step = self.interpolate_partial(
                current_joint_values.copy(),
                target_position,
                self.rotation_joints_indices,
                self.step_size_rotation_joints,
                other_joints_values=target_position
            )
            return next_step
        else:
            next_step = self.interpolate_partial(
                current_joint_values.copy(),
                target_position,
                self.flexion_joints_indices,
                self.step_size_flexion_joints,
                other_joints_values=current_joint_values.copy()
            )
            return next_step
        
    # Mirror the joint positions for mirrored joints based on the configuration
    def mirror_position(self, joint_names, target_position):
        mirrored_indices = [
            i
            for i, name in enumerate(joint_names)
            if any(base in name for base in self.MIRRORED_BASES)
        ]
        for i in mirrored_indices:
            target_position[i] = -target_position[i]
        return target_position
        
    # Check if two lists of joint angles are equal within a tolerance, skipping index 1
    def joint_angles_equal(self, list1, list2):
        if len(list1) != len(list2):
            return False
        for i, (a, b) in enumerate(zip(list1, list2)):
            if i == 1:
                continue  # Skip index 1
            if abs(a - b) > self.tolerance:
                return False
        return True

    # Extract joint values from a list of joint state dictionaries based on the provided joint names
    def extract_joint_values(self, all_joint_states, joint_names):
        for joint_dict in all_joint_states:
            if all(name in joint_dict for name in joint_names):
                return [joint_dict[name] for name in joint_names]
        return []  # Not found
    
    # Check if the current joint angles are at the target position for the specified indices
    def joints_at_pose(self, current, indices, target_position):
        result = all(abs(current[i] - target_position[i]) < self.tolerance for i in indices)
        return result
    
    # Interpolate partial joint positions towards the target position for specified indices
    def interpolate_partial(self, current, target, indices, step_size, other_joints_values):
        next_step = other_joints_values[:]  # Set all values to 0.0 initially
        for i in indices:
            delta = target[i] - current[i]
            if abs(delta) > step_size:
                next_step[i] = current[i] + step_size * (1 if delta > 0 else -1)
            else:
                next_step[i] = target[i]
        return next_step
    
    # Publish a joint trajectory message for the given positions using the provided publisher
    def publish_joint_trajectory(
        self, target_positions, publisher, joint_names=None):
        """Publishes a joint trajectory message for the given positions using the provided publisher."""
        if joint_names is None:
            joint_names = list(self.joint_states.keys())

        if not joint_names:
            self.get_logger().error("No joint names available. Cannot publish trajectory.")
            return

        if not target_positions:
            self.get_logger().error("No trajectory points available to publish.")
            return

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(joint_names)
        point.accelerations = [0.0] * len(joint_names)
        time_in_sec = self.dt
        sec = int(time_in_sec)
        nanosec = int((time_in_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        trajectory_msg.points.append(point)
        publisher.publish(trajectory_msg)
    
    # Get joint states for the specified number of arms
    def get_joint_states(self, arms_count):
        """Always return a list of joint state dicts, one per arm."""
        joint_states = self.joint_states
        if arms_count <= 1:
            return [joint_states]  # Always a list, even for single arm

        # Multi-arm: split joint_states into self.arms_count chunks
        joint_names = list(joint_states.keys())
        values = list(joint_states.values())
        chunk_size = len(joint_names) // arms_count
        joint_states_per_arm = []
        for i in range(arms_count):
            start = i * chunk_size
            end = (i + 1) * chunk_size
            arm_joint_names = joint_names[start:end]
            arm_joint_values = values[start:end]
            arm_joint_dict = dict(zip(arm_joint_names, arm_joint_values))
            joint_states_per_arm.append(arm_joint_dict)
        return joint_states_per_arm
    
    # Retrieve parameter values from the node
    def get_param_values(self, controller_ns, param_name):
        """Retrieve parameter values from the node."""
        param_client = AsyncParameterClient(self, controller_ns)
        future = param_client.get_parameters([param_name])
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().values:
            param_value = future.result().values[0]
            joint_names = list(param_value.string_array_value)
            return joint_names
        
    # Get topic names and types, filtering by a given name pattern.
    def get_topic_names_and_types_test(self, by_name):
        """This base class retrieves topic names by a given name.
        Args:
            by_name (str): The name of the topic to retrieve.
        Returns:
            list: A list of topic names and types matching the given name.
        """
        pattern = re.compile(by_name.replace("*", ".*"))
        topics_and_types = self.get_topic_names_and_types()
        matches = [(topic, types) for topic, types in topics_and_types if pattern.fullmatch(topic)]
        self.arms_count = len(matches)
        return matches
    
    # Acrtivate all joint trajectory controllers
    def switch_to_joint_trajectory_controllers(self):
        """Switch to all controllers that start with the given prefix."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                "SwitchController service not available.", throttle_duration_sec=10.0
            )
            return
        
        req = SwitchController.Request()
        req.deactivate_controllers = [name for controller in self.all_controllers.get(self.previous_controller, []) for name, active in controller.items()]
        req.activate_controllers = [name for controller in self.all_controllers.get('joint_trajectory_controller', []) for name, active in controller.items()]
        req.strictness = 1  # STRICT
        self.switch_controller_client.call_async(req)

    # Activate the previous active controllers
    def switch_to_previous_controllers(self):
        """Deactivate all joint trajectory controllers."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "SwitchController service not available.", throttle_duration_sec=10.0
            )
            return

        req = SwitchController.Request()
        deaktivate_controllers = [name for controller in self.all_controllers.get("joint_trajectory_controller", []) for name, active in controller.items()]
        req.deactivate_controllers = deaktivate_controllers
        req.strictness = 1
        if self.previous_controller is not None:
            req.activate_controllers =  [name for controller in self.all_controllers.get(self.previous_controller, []) for name, active in controller.items()]
        self.switch_controller_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPredefinedPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()