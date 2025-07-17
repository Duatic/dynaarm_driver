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


    def __init__(self, node, controllers_config):
        self.node = node
        self.controllers = {}

        self.current_active_controller = None
        self._is_freeze_active = False

        self.controller_client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.controller_whitelist = [
            name for name, props in controllers_config.items() if props["whitelisted"]
        ]

        self._active_controllers = set()      
        self._is_freeze_active = False        
        self._found_controllers_by_base = {base: [] for base in self.controller_whitelist}
        self._run_once = False
        self.node.create_timer(0.1, self._get_all_controllers)

    def get_all_controllers(self):
        """Returns the found controllers by base. May be empty if timer hasn't run yet."""
        return self._found_controllers_by_base
    
    def get_active_controllers(self):
        """Returns a list of currently active controllers. May be empty if timer hasn't run yet."""
        return self._active_controllers
    
    def is_freeze_active(self):
        """Checks if the freeze controller is currently active. Defaults to False if not yet determined."""
        return self._is_freeze_active

    def is_controller_data_ready(self):
        """Returns True if controller data has been fetched at least once."""
        return self._run_once

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

                    # Reset found controllers and active controllers
                    for base in self._found_controllers_by_base:
                        self._found_controllers_by_base[base] = []
                    self._active_controllers.clear()

                    # Populate found controllers by base name
                    for controller in response.controller:
                        for base in self.controller_whitelist:
                            if controller.name.startswith(base):
                                self._found_controllers_by_base[base].append(
                                    {controller.name: controller.state}
                                )

                                if controller.state == "active":
                                    self._active_controllers.add(base)
                        
                        if controller.name.startswith("freeze_controller"):
                            if controller.state == "active":
                                self._is_freeze_active = True
                            else:
                                self._is_freeze_active = False                        
                            
                    self._run_once = True   
                    
                except Exception as e:
                    self.node.get_logger().error(f"Error fetching controllers: {e}")

        future.add_done_callback(callback)
