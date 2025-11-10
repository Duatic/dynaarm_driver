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

from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers, SwitchController

from dynaarm_extensions.duatic_helpers.duatic_controller_helper import (
    DuaticControllerHelper,
)
from dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper
from dynaarm_extensions.duatic_helpers.duatic_jtc_helper import DuaticJTCHelper


class MoveToPredefinedPositionNode(Node):

    MIRRORED_BASES = {"shoulder_rotation", "forearm_rotation", "wrist_rotation"}

    def __init__(self):
        super().__init__("motion_to_predefined_position_node")

        self.declare_parameter("robot_configuration", "dynaarm")  # Default configuration
        self.robot_configuration = self.get_parameter("robot_configuration").value

        # Service clients
        self.list_controllers_client = self.create_client(
            ListControllers, "controller_manager/list_controllers"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, "controller_manager/switch_controller"
        )

        # Subscriptions
        self.create_subscription(Bool, "move_home", self.move_home_callback, 10)
        self.create_subscription(Bool, "move_sleep", self.move_sleep_callback, 10)

        self.sleep_position_dynaarm = [0.0, -1.7, 3.1, -1.5708, 0.0, 0.0]
        self.home_position_dynaarm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.home_position_alpha = [1.5708, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.sleep_position_alpha = [1.5708, 0.77, 0.0, 0.0, 0.0, 0.0]

        self.moving_states = {}
        self.home = False  # Initialize home position flag
        self.sleep = False  # Initialize sleep position flag

        self.flexion_joints_indices = [1, 2, 4]  # Indices of flexion joints
        self.rotation_joints_indices = [0, 3, 5]  # Indices of rotation joints
        self.tolerance = 0.01  # Tolerance for joint angle comparison
        self.controller_active = False  # Flag to check if the controller is active
        self.joint_trajectory_publishers = (
            {}
        )  # Dictionary to hold publishers for joint trajectory topics
        self.topic_to_joint_names = {}  # Dictionary to map topics to joint names
        self.topic_to_commanded_positions = (
            {}
        )  # Dictionary to hold commanded positions for each topic
        self.prefix_to_joints = {}  # Dictionary to map prefixes to joint names
        self.previous_active_controllers = []

        self.movement_handlers = {
            ("dynaarm", "home"): self.move_home_dynaarm,
            ("dynaarm", "sleep"): self.move_sleep_dynaarm,
            ("dynaarm_dual", "home"): self.move_home_dynaarm,
            ("dynaarm_dual", "sleep"): self.move_sleep_dynaarm,
            ("alpha", "home"): self.move_home_alpha,
            ("alpha", "sleep"): self.move_sleep_alpha,
            ("dynaarm_flip", "home"): self.move_home_dynaarm,
            ("dynaarm_flip", "sleep"): self.move_home_dynaarm,
        }

        # Helper classes
        self.duatic_controller_helper = DuaticControllerHelper(self)
        self.duatic_controller_helper.wait_for_controller_data()

        self.duatic_robots_helper = DuaticRobotsHelper(self)
        self.arms_count = self.duatic_robots_helper.wait_for_robot()

        duatic_jtc_helper = DuaticJTCHelper(self)
        self.arms = self.duatic_robots_helper.get_component_names("arm")
        found_topics = duatic_jtc_helper.find_topics_for_controller(
            "joint_trajectory_controller", "joint_trajectory", self.arms
        )

        response = duatic_jtc_helper.process_topics_and_extract_joint_names(found_topics)
        self.topic_to_joint_names = response[0]
        self.topic_to_commanded_positions = response[1]

        # Set the dt value depending on the robot configuration
        self.dt = self.duatic_robots_helper.get_dt()

        # Target velocities for flexion and rotation joints
        self.target_velocity_flexion = 0.8  # 0.8 rad/s = ~45.8°/s
        self.target_velocity_rotation = 1.2  # 1.2 rad/s = ~68.2°/s

        # Calculate step sizes based on dt to achieve target velocities
        self.step_size_flexion_joints = self.target_velocity_flexion * self.dt
        self.step_size_rotation_joints = self.target_velocity_rotation * self.dt

        # Create publishers for each joint trajectory topic
        for topic in self.topic_to_joint_names.keys():
            self.joint_trajectory_publishers[topic] = self.create_publisher(
                JointTrajectory, topic, 10
            )
            self.get_logger().debug(f"Created publisher for topic: {topic}")

        self.create_timer(self.dt, self.control_loop)

    def reset_state(self):
        """Reset the state of the node."""
        if self.sleep or self.home:
            return

        for topic, _ in self.topic_to_joint_names.items():
            self.moving_states[topic] = "none"

    def move_home_callback(self, msg):
        self.home = msg.data
        self.sleep = False  # Reset sleep flag when moving to home
        self.reset_state()

    def move_sleep_callback(self, msg):
        self.sleep = msg.data
        self.home = False  # Reset home flag when moving to sleep
        self.reset_state()

    def control_loop(self):
        """Main control loop to handle movement commands."""
        # If freeze is active, do not execute any movement
        if self.duatic_controller_helper.is_freeze_active():
            return

        if self.home or self.sleep:

            if not self.controller_active:
                self.controller_active = True
                self.switch_to_joint_trajectory_controllers()

            # O(1) dictionary lookup instead of multiple string comparisons
            action_key = (self.robot_configuration, "home" if self.home else "sleep")
            handler = self.movement_handlers.get(action_key)
            if handler:
                handler()
        else:
            if (
                self.controller_active
                and "joint_trajectory_controller" not in self.previous_active_controllers
            ):
                self.controller_active = False
                self.switch_to_previous_controllers()
            else:
                self.previous_active_controllers = list(
                    self.duatic_controller_helper.get_active_controllers()
                )

    def move_home_dynaarm(self):
        """Move to home position for DynAarm configuration."""
        self._move_generic(
            home_position=self.home_position_dynaarm.copy(),
            sleep_position=self.sleep_position_dynaarm.copy(),
            use_mirroring=False,
        )

    def move_home_alpha(self):
        """Move to home position for Alpha configuration (dual arm)."""
        self._move_generic(
            home_position=self.home_position_alpha.copy(),
            sleep_position=self.sleep_position_alpha.copy(),
            use_mirroring=True,
        )

    def move_sleep_dynaarm(self):
        """Move to sleep position for DynAarm configuration (single arm)."""
        self._move_generic(
            home_position=self.home_position_dynaarm.copy(),
            sleep_position=self.sleep_position_dynaarm.copy(),
            use_mirroring=False,
        )

    def move_sleep_alpha(self):
        """Move to sleep position for Alpha configuration (dual arm)."""
        self._move_generic(
            home_position=self.home_position_alpha.copy(),
            sleep_position=self.sleep_position_alpha.copy(),
            use_mirroring=True,
        )

    def _move_generic(self, home_position, sleep_position, use_mirroring):
        """Generic sleep movement logic with 3-phase state machine."""
        arm_index = 0

        for topic, joint_names in self.topic_to_joint_names.items():
            # Initialize state for this topic/arm if not exists
            if topic not in self.moving_states:
                self.moving_states[topic] = "none"
                self.get_logger().debug(f"Arm {arm_index}: Initialized state to 'none'")

            all_joint_states = self.duatic_robots_helper.get_joint_states()
            current_joint_values = [
                all_joint_states.get(joint_name, 0.0) for joint_name in joint_names
            ]

            # Determine target position based on mirroring
            if use_mirroring and arm_index == 1:  # Second arm gets mirrored positions
                target_home = self.mirror_position(joint_names, home_position.copy())
                target_sleep = self.mirror_position(joint_names, sleep_position.copy())
            else:
                target_home = home_position.copy()
                target_sleep = sleep_position.copy()

            # Create intermediate position: home position with sleep rotation joints
            target_sleep_rotation = target_home.copy()
            for i in self.rotation_joints_indices:
                target_sleep_rotation[i] = target_sleep[i]

            # State machine logic per arm
            at_home_flexion = self.joints_at_pose(
                current_joint_values, self.flexion_joints_indices, target_home
            )
            at_sleep_flexion = self.joints_at_pose(
                current_joint_values, self.flexion_joints_indices, target_sleep
            )
            at_home_rotation = self.joints_at_pose(
                current_joint_values, self.rotation_joints_indices, target_home
            )
            at_sleep_rotation = self.joints_at_pose(
                current_joint_values, self.rotation_joints_indices, target_sleep
            )

            current_state = self.moving_states[topic]
            commanded_positions = current_joint_values.copy()

            self.get_logger().debug(
                f"Arm {arm_index}: State={current_state}, at_home={at_home_flexion},  at_sleep={at_sleep_flexion}, at_home_rotation={at_home_rotation}, at_sleep_rotation={at_sleep_rotation}"
            )

            if self.sleep:
                # If robot is at target position
                if at_sleep_flexion and at_sleep_rotation:
                    self.get_logger().debug(
                        f"Arm {arm_index}: Already at sleep position, no movement needed"
                    )
                    self.moving_states[topic] = "at_sleep"
                # If robot is at target rotations
                elif at_sleep_rotation:
                    self.get_logger().debug(
                        f"Arm {arm_index}: Rotation already correct, going to flexion phase"
                    )
                    self.moving_states[topic] = "going_sleep_flexion"
                # If at home flexion
                elif at_home_flexion:
                    self.get_logger().debug(f"Arm {arm_index}: At home, moving to sleep position")
                    self.moving_states[topic] = "going_sleep_rotation"
                # If robot is not at target rotation
                elif not at_sleep_rotation:
                    self.get_logger().debug(
                        f"Arm {arm_index}: Not at sleep rotation, moving home first"
                    )
                    self.moving_states[topic] = "going_home_flexion"
                else:
                    self.moving_states[topic] = "error"
            elif self.home:
                # If robot is at target position
                if at_home_flexion and at_home_rotation:
                    self.get_logger().debug(
                        f"Arm {arm_index}: Already at home position, no movement needed"
                    )
                    self.moving_states[topic] = "at_home"
                # If robot is at target rotations
                elif not at_home_flexion:
                    self.get_logger().debug(
                        f"Arm {arm_index}: Rotation already correct, going to flexion phase"
                    )
                    self.moving_states[topic] = "going_home_flexion"
                # If at home flexion
                elif not at_home_rotation:
                    self.get_logger().debug(
                        f"Arm {arm_index}: At home flexion, moving to rotation phase"
                    )
                    self.moving_states[topic] = "going_home_rotation"
                else:
                    self.moving_states[topic] = "error"

            current_state = self.moving_states[topic]

            if current_state == "at_sleep" or current_state == "at_home":
                pass
            elif current_state == "going_sleep_flexion":
                commanded_positions = self.move_flexion_only(target_sleep, current_joint_values)
            elif current_state == "going_sleep_rotation":
                commanded_positions = self.move_rotation_only(
                    target_sleep_rotation, current_joint_values
                )
            elif current_state == "going_home_flexion":
                commanded_positions = self.move_flexion_only(target_home, current_joint_values)
            elif current_state == "going_home_rotation":
                commanded_positions = self.move_rotation_only(target_home, current_joint_values)
            else:
                self.get_logger().error(f"Arm {arm_index}: Invalid state '{current_state}'")
                commanded_positions = current_joint_values.copy()

            self.topic_to_commanded_positions[topic] = commanded_positions
            arm_index += 1

        # Publish trajectories for all arms
        for topic, publisher in self.joint_trajectory_publishers.items():

            if self.moving_states[topic] == "at_sleep" or self.moving_states[topic] == "at_home":
                continue

            self.publish_joint_trajectory(
                self.topic_to_commanded_positions[topic],
                publisher=publisher,
                joint_names=self.topic_to_joint_names[topic],
            )

    def mirror_position(self, joint_names, target_position):
        # Mirror the joint positions for mirrored joints based on the configuration
        mirrored_indices = [
            i
            for i, name in enumerate(joint_names)
            if any(base in name for base in self.MIRRORED_BASES)
        ]
        for i in mirrored_indices:
            target_position[i] = -target_position[i]
        return target_position

    def joints_at_pose(self, current, indices, target_position):
        # Check if the current joint angles are at the target position for the specified indices
        result = all(abs(current[i] - target_position[i]) < self.tolerance for i in indices)
        return result

    def interpolate_partial(self, current, target, indices, step_size):
        """Interpolate the current joint values towards the target for specified indices."""
        next_step = current[:]  # Start with current joint values
        for i in indices:
            delta = target[i] - current[i]
            if abs(delta) > step_size:
                next_step[i] = current[i] + step_size * (1 if delta > 0 else -1)
            else:
                next_step[i] = target[i]
        return next_step

    def move_flexion_only(self, target_position, current_joint_values):
        next_step = self.interpolate_partial(
            current_joint_values.copy(),
            target_position,
            self.flexion_joints_indices,
            self.step_size_flexion_joints,
        )
        return next_step

    def move_rotation_only(self, target_position, current_joint_values):
        next_step = self.interpolate_partial(
            current_joint_values.copy(),
            target_position,
            self.rotation_joints_indices,
            self.step_size_rotation_joints,
        )
        return next_step

    def publish_joint_trajectory(self, target_positions, publisher, joint_names=None):
        """Publishes a joint trajectory message for the given positions using the provided publisher."""
        if joint_names is None:
            joint_names = list(self.duatic_robots_helper.get_joint_states().keys())

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

    def switch_to_joint_trajectory_controllers(self):
        """Switch to all joint trajectory controllers if not already active."""
        all_controllers = self.duatic_controller_helper.get_all_controllers()
        controllers_to_activate = []
        controllers_to_deactivate = []

        # Single pass: collect all JTCs and check if any is active, collect others to deactivate
        for controller_type, controllers in all_controllers.items():
            for controller in controllers:
                for name, active_state in controller.items():
                    if (
                        controller_type == "joint_trajectory_controller"
                        and active_state == "inactive"
                    ):
                        controllers_to_activate.append(name)
                    elif (
                        controller_type == "joint_trajectory_controller"
                        and active_state == "active"
                    ):
                        pass
                    elif active_state == "inactive":
                        pass
                    else:
                        controllers_to_deactivate.append(name)

        self.duatic_controller_helper.switch_controller(
            controllers_to_activate, controllers_to_deactivate
        )

    def switch_to_previous_controllers(self):
        """Deactivate all joint trajectory controllers."""
        all_controllers = self.duatic_controller_helper.get_all_controllers()
        controllers_to_activate = []
        controllers_to_deactivate = []

        if not self.previous_active_controllers or len(self.previous_active_controllers) <= 0:
            self.get_logger().debug(
                "No previous controller state found. Deactivating joint trajectory controllers."
            )
            # Just deactivate all active joint trajectory controllers
            for controller_type, controllers in all_controllers.items():
                for controller in controllers:
                    for name, active_state in controller.items():
                        if "joint_trajectory_controller" in name and active_state == "active":
                            controllers_to_deactivate.append(name)
        else:
            # Single pass: collect all JTCs and check if any is active, collect others to deactivate
            for controller_type, controllers in all_controllers.items():
                for controller in controllers:
                    for name, active_state in controller.items():
                        if "joint_trajectory_controller" in name and active_state == "active":
                            if name not in self.previous_active_controllers:
                                controllers_to_deactivate.append(name)
                        elif name in self.previous_active_controllers:
                            controllers_to_activate.append(name)
                        else:
                            pass

        self.get_logger().debug(
            f"Switching to previous controllers: {controllers_to_activate} and deactivating: {controllers_to_deactivate}"
        )
        self.duatic_controller_helper.switch_controller(
            controllers_to_activate, controllers_to_deactivate
        )


def main(args=None):

    rclpy.init(args=args)

    node = MoveToPredefinedPositionNode()
    node.get_logger().info("Move to Predefined Position Node is running...")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
