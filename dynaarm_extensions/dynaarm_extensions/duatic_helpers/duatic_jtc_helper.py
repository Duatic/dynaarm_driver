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

import re
import rclpy
from dynaarm_extensions.duatic_helpers.duatic_param_helper import DuaticParamHelper


class DuaticJTCHelper:
    """Helper class for Joint Trajectory Controller topic discovery and management."""

    def __init__(self, node, arms_count):
        self.node = node
        self.arms_count = arms_count

        if arms_count <= 0:
            self.node.get_logger().error("arms_count must be greater than 0")
            raise ValueError("arms_count must be greater than 0")

        self.duatic_param_helper = DuaticParamHelper(self.node)

    def process_topics_and_extract_joint_names(self, found_topics):

        topic_to_joint_names = {}  # Dictionary to map topics to joint names
        topic_to_commanded_positions = {}  # Dictionary to hold commanded positions for each topic

        # Discover all topics and joint names, extract prefix
        for topic, types in found_topics:

            # Extract prefix from topic name
            # e.g. /joint_trajectory_controller_arm_1/joint_trajectory -> arm_1
            controller_ns = topic.split("/")[1]
            param_result = self.duatic_param_helper.get_param_values(controller_ns, "joints")
            if param_result is None or not param_result:
                self.node.get_logger().error(f"Parameter 'joints' not found for {controller_ns}")
                break

            joint_names = list(param_result[0].string_array_value)

            self.node.get_logger().debug(
                f"Retrieved joint names for {controller_ns}: {joint_names}"
            )
            if joint_names:
                topic_to_joint_names[topic] = joint_names
                topic_to_commanded_positions[topic] = {}
            else:
                print("Parameter not found or empty for topic", topic)

        return topic_to_joint_names, topic_to_commanded_positions

    # Get topic names and types, filtering by a given name pattern.
    def get_topic_names_and_types_function(self, by_name):
        # Convert wildcard pattern to regex pattern
        # Handle the specific case where we search for controller patterns
        if "*" in by_name:
            # Convert patterns like "joint_trajectory_controller*/joint_trajectory"
            # to match "/joint_trajectory_controller_arm_left/joint_trajectory"
            pattern_str = by_name.replace("*", ".*")
            # Ensure we match from the beginning of the topic name
            if not pattern_str.startswith("/"):
                pattern_str = "/" + pattern_str
            pattern = re.compile(pattern_str)
        else:
            pattern = re.compile(re.escape(by_name))

        topics_and_types = self.node.get_topic_names_and_types()
        self.node.get_logger().debug(f"Found topics and types: {topics_and_types}")

        matches = [(topic, types) for topic, types in topics_and_types if pattern.search(topic)]
        self.node.get_logger().debug(f"Filtered topics matching '{by_name}': {matches}")
        return matches

    def find_topics_for_controller(self, controller_name, identifier):
        found_topics = []
        max_retries = 100
        retry_count = 0

        # Construct the search pattern
        search_pattern = f"/{controller_name}*/{identifier}"

        # Find all topics matching the controller name
        while len(found_topics) < self.arms_count and retry_count < max_retries:
            found_topics = self.get_topic_names_and_types_function(search_pattern)

            if len(found_topics) >= self.arms_count:
                break

            self.node.get_logger().info(
                f"Found {len(found_topics)} topics, expecting {self.arms_count}. Retrying...",
                throttle_duration_sec=10,
            )
            rclpy.spin_once(self.node, timeout_sec=0.5)
            retry_count += 1

        if len(found_topics) < self.arms_count:
            self.node.get_logger().error(
                f"Expected {self.arms_count} controller topics, but found {len(found_topics)}"
            )
            self.node.get_logger().error(
                f"Available topics: {[topic for topic, _ in self.node.get_topic_names_and_types()]}"
            )
            raise RuntimeError(
                f"Expected {self.arms_count} controller topics, but found {len(found_topics)}"
            )

        return found_topics
