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
from controller_manager_msgs.srv import ListControllers, SwitchController


class DuaticControllerHelper:

    def __init__(self, node, robot_namespace=""):
        self.node = node

        # Resolve namespace from parameter if not explicitly provided
        if not robot_namespace:
            robot_namespace = self._resolve_namespace()

        # Clean up namespace (remove trailing slashes)
        self.robot_namespace = robot_namespace.rstrip("/") if robot_namespace else ""

        self.controller_whitelist = [
            "freedrive_controller",
            "joint_trajectory_controller",
            "cartesian_pose_controller",
            "mecanum_drive_controller",
        ]

        self.active_low_level_controllers = []
        self._is_freeze_active = False

        # Only store controllers that are actually found
        self._found_controllers_by_base = {}

        self._run_once = False

        # Construct service paths with namespace
        base_path = f"/{self.robot_namespace}" if self.robot_namespace else ""

        self.controller_client = self.node.create_client(
            ListControllers, f"{base_path}/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.node.create_client(
            SwitchController, f"{base_path}/controller_manager/switch_controller"
        )

        self.node.create_timer(0.1, self._get_all_controllers)

    def _resolve_namespace(self):
        """
        Resolve namespace from multiple sources in priority order.

        Priority order:
        1. Node namespace (if set and not root)
        2. Empty string (default - for backward compatibility)
        3. URDF robot name (optional, from robot_description parameter)
        """
        # Priority 1: Node namespace
        node_ns = self.node.get_namespace()
        if node_ns and node_ns != "/":
            cleaned_ns = node_ns.strip("/")
            self.node.get_logger().info(f"Using namespace from node: {cleaned_ns}")
            return cleaned_ns

        # Priority 2: Default to empty string (backward compatible)
        # This allows users to explicitly NOT use a namespace

        # Priority 3 (Optional): Try to extract robot name from URDF
        # Only if explicitly needed - call _get_robot_name_from_urdf()
        # Disabled by default to maintain backward compatibility
        # Uncomment the following lines to enable URDF-based namespace detection:
        #
        # urdf_robot_name = self._get_robot_name_from_urdf()
        # if urdf_robot_name:
        #     self.node.get_logger().info(f"Using namespace from URDF: {urdf_robot_name}")
        #     return urdf_robot_name

        return ""

    def _get_robot_name_from_urdf(self):
        """
        Extract robot name from URDF robot_description parameter.

        Returns empty string if not found or on error.
        """
        try:
            from dynaarm_extensions.duatic_helpers.duatic_param_helper import DuaticParamHelper
            import xml.etree.ElementTree as ET

            param_helper = DuaticParamHelper(self.node)
            urdf_values = param_helper.get_param_values(
                "robot_state_publisher", "robot_description"
            )

            if urdf_values and urdf_values[0].string_value:
                urdf_xml = urdf_values[0].string_value

                # Parse URDF XML to extract robot name
                root = ET.fromstring(urdf_xml)
                robot_name = root.attrib.get("name", "")

                if robot_name:
                    self.node.get_logger().debug(f"Found robot name in URDF: {robot_name}")
                    return robot_name

        except Exception as e:
            self.node.get_logger().debug(f"Could not extract robot name from URDF: {e}")

        return ""

    def get_all_controllers(self, matching_names=None):
        """
        Returns the found controllers by base.

        If matching_names is provided, returns a dict with only those controllers.
        """
        if not matching_names:
            return self._found_controllers_by_base

        # Filter controllers by matching_names and return only controller names
        filtered = []
        for base, controllers in self._found_controllers_by_base.items():
            if base in matching_names:
                # controllers is a list of dicts like [{name: state}]
                filtered.extend([list(ctrl.keys())[0] for ctrl in controllers])
        return filtered

    def get_active_controllers(self):
        """Returns a list of currently active controllers. May be empty if timer hasn't run yet."""
        return self.active_low_level_controllers

    def is_freeze_active(self):
        """Checks if the freeze controller is currently active. Defaults to False if not yet determined."""
        return self._is_freeze_active

    def is_controller_data_ready(self):
        """Returns True if controller data has been fetched at least once."""
        return self._run_once

    def switch_controller(self, activate_controllers, deactivate_controllers):

        if not activate_controllers and not deactivate_controllers:
            self.node.get_logger().debug(
                "No controllers to activate or deactivate. Skipping switch operation."
            )
            return

        # Filter out controllers that are already in the desired state
        active_controllers = self.get_active_controllers()

        # Only activate controllers that are not already active
        controllers_to_activate = [c for c in activate_controllers if c not in active_controllers]

        # Only deactivate controllers that are currently active
        controllers_to_deactivate = [c for c in deactivate_controllers if c in active_controllers]

        # If nothing needs to change, skip the operation
        if not controllers_to_activate and not controllers_to_deactivate:
            self.node.get_logger().info(
                "All requested controllers are already in the desired state. Skipping switch."
            )
            return

        req = SwitchController.Request()
        req.activate_controllers = controllers_to_activate
        req.deactivate_controllers = controllers_to_deactivate

        req.strictness = 1

        future = self.switch_controller_client.call_async(req)

        def callback(future):
            try:
                response = future.result()
                if response.ok:
                    self._get_all_controllers()
                    self.node.get_logger().info(
                        f"Successfully switched controllers: activated={controllers_to_activate}, deactivated={controllers_to_deactivate}"
                    )
                else:
                    self.node.get_logger().error(
                        "Failed to switch to new controller", throttle_duration_sec=10.0
                    )
            except Exception as e:
                self.node.get_logger().error(
                    f"Error switching controllers: {e}", throttle_duration_sec=10.0
                )

        future.add_done_callback(callback)

    def wait_for_controller_loaded(self, controller_name, timeout=60.0):
        """Checks if a specific controller is loaded."""
        if not self.wait_for_controller_data(timeout_sec=timeout):
            return False

        found_controllers = self._found_controllers_by_base
        for base, controllers in found_controllers.items():
            for ctrl in controllers:
                if controller_name in ctrl:
                    return True

        return False

    def wait_for_controller_data(self, timeout_sec=20.0):
        """Wait for controller data to be available, returns True if successful."""
        import time

        start_time = time.time()
        while not self._run_once and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self._run_once

    def _get_all_controllers(self):
        """Fetches all controllers and their states."""
        if not self.controller_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                "Controller manager service not available.", throttle_duration_sec=10.0
            )
            return

        req = ListControllers.Request()
        future = self.controller_client.call_async(req)

        def callback(future):
            if future.done():
                try:
                    response = future.result()

                    # Get current controllers from response
                    current_controllers = {ctrl.name: ctrl.state for ctrl in response.controller}

                    # Clear active controllers
                    self.active_low_level_controllers.clear()

                    # Rebuild found_controllers_by_base with only currently existing controllers
                    new_found_controllers = {}

                    # Check each controller in the whitelist
                    for base in self.controller_whitelist:
                        controllers_for_base = []

                        # Find all controllers that match this base
                        for controller_name, state in current_controllers.items():
                            if controller_name.startswith(base):
                                controllers_for_base.append({controller_name: state})

                                # Add to active list if active
                                if state == "active":
                                    self.active_low_level_controllers.append(controller_name)

                        # Only add to found_controllers if we actually found controllers for this base
                        if controllers_for_base:
                            new_found_controllers[base] = controllers_for_base

                    # Replace the old dictionary with the new one
                    self._found_controllers_by_base = new_found_controllers

                    # Handle freeze controller separately
                    freeze_active = False
                    for controller_name, state in current_controllers.items():
                        if controller_name.startswith("freeze_controller"):
                            if state == "active":
                                freeze_active = True
                            break

                    self._is_freeze_active = freeze_active

                    # Log controller changes (optional)
                    if self._run_once:
                        available_bases = list(self._found_controllers_by_base.keys())
                        missing_bases = [
                            base
                            for base in self.controller_whitelist
                            if base not in available_bases
                        ]

                        if missing_bases:
                            self.node.get_logger().debug(
                                f"Controller bases not found: {missing_bases}"
                            )

                    self._run_once = True

                except Exception as e:
                    self.node.get_logger().error(f"Error fetching controllers: {e}")

        future.add_done_callback(callback)
