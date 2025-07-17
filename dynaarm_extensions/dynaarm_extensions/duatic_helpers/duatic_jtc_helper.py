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

import time
import rclpy
import subprocess
import json

class DuaticJTCHelper:
    """Helper class for Joint Trajectory Controller topic discovery and management."""

    def __init__(self, node, arms_count):        
        self.node = node
        self.arms_count = arms_count

        self.topic_prefix = "/joint_trajectory_controller"

        if arms_count <= 0:
            self.node.get_logger().error("arms_count must be greater than 0")
            raise ValueError("arms_count must be greater than 0")       

    def process_topics_and_extract_joint_names(self, found_topics):

        topic_to_joint_names = {}  # Dictionary to map topics to joint names
        topic_to_commanded_positions = ({})  # Dictionary to hold commanded positions for each topic
    
        # Discover all topics and joint names, extract prefix
        for topic, types in found_topics: 
            
            # Extract prefix from topic name
            # e.g. /joint_trajectory_controller_arm_1/joint_trajectory -> arm_1
            controller_ns = topic.split("/")[1]
            joint_names = self._get_param_values(controller_ns, "joints")
            self.node.get_logger().debug(f"Retrieved joint names for {controller_ns}: {joint_names}")
            if joint_names:
                topic_to_joint_names[topic] = joint_names
                topic_to_commanded_positions[topic] = [0.0] * len(joint_names)
            else:
                print("Parameter not found or empty for topic", topic)

        return topic_to_joint_names, topic_to_commanded_positions

    def get_joint_trajectory_topics(self):

        found_topics = {}        

        # Find all joint trajectory topics matching the prefix
        while len(found_topics) != self.arms_count:
            found_topics = self.node.get_topic_names_and_types_function(
                f"{self.topic_prefix}*/joint_trajectory"
            )
            rclpy.spin_once(self.node, timeout_sec=0.01)
        
        if not found_topics:
            self.node.get_logger().error("No joint trajectory topics found")
            raise RuntimeError("No joint trajectory topics found")
        
        return found_topics    
    
    def _get_param_values(self, controller_ns, param_name):
        """Get parameter values using ros2 CLI to avoid executor conflicts."""
        try:
            # Use ros2 param get command to avoid executor conflicts
            cmd = ["ros2", "param", "get", f"/{controller_ns}", param_name]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            
            if result.returncode == 0:
                # Parse the output - it should be in YAML format
                output = result.stdout.strip()
                self.node.get_logger().debug(f"Raw parameter output for {controller_ns}/{param_name}: {output}")
                
                # Extract the array values from the YAML output
                # Expected format: "['joint1', 'joint2', 'joint3']" or similar
                if "[" in output and "]" in output:
                    # Extract the list part
                    start = output.find("[")
                    end = output.find("]") + 1
                    list_str = output[start:end]
                    
                    # Parse as Python list (safely)
                    import ast
                    joint_names = ast.literal_eval(list_str)
                    return joint_names
                else:
                    self.node.get_logger().warn(f"Unexpected parameter format: {output}")
                    
            else:
                self.node.get_logger().warn(f"Failed to get parameter {controller_ns}/{param_name}: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self.node.get_logger().warn(f"Timeout getting parameter {controller_ns}/{param_name}")
        except Exception as e:
            self.node.get_logger().warn(f"Error getting parameter {controller_ns}/{param_name}: {e}")

        return None