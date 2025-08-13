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

    def __init__(self, node):
        self.node = node

        self.controller_whitelist = [
            "freedrive_controller",
            "joint_trajectory_controller",
            "dynaarm_pose_controller",
            "mecanum_drive_controller"
        ]

        self.active_low_level_controllers = []
        self._is_freeze_active = False

        # Keep track of all controllers we've ever seen
        self._all_known_controllers = set()
        
        # Only store controllers that are actually found
        self._found_controllers_by_base = {}       
        
        self._run_once = False

        self.controller_client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.node.create_timer(0.1, self._get_all_controllers)

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

        req = SwitchController.Request()
        req.activate_controllers = activate_controllers
        req.deactivate_controllers = deactivate_controllers

        req.strictness = 1

        future = self.switch_controller_client.call_async(req)

        def callback(future):
            try:
                response = future.result()
                if response.ok:
                    self._get_all_controllers()
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
                                
                                # Track all known controllers
                                self._all_known_controllers.add(controller_name)
                                
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
                        missing_bases = [base for base in self.controller_whitelist if base not in available_bases]
                        
                        if missing_bases:
                            self.node.get_logger().debug(f"Controller bases not found: {missing_bases}")

                    self._run_once = True

                except Exception as e:
                    self.node.get_logger().error(f"Error fetching controllers: {e}")

        future.add_done_callback(callback)