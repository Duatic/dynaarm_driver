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
import time
import threading
from controller_manager_msgs.srv import ListControllers, SwitchController
from dynaarm_gamepad_interface.controllers.joint_trajectory_controller import (
    JointTrajectoryController,
)
from dynaarm_gamepad_interface.controllers.position_controller import PositionController
from dynaarm_gamepad_interface.controllers.cartesian_controller import CartesianController
from dynaarm_gamepad_interface.controllers.freedrive_controller import FreedriveController


class DynAarmControllerManager:

    # Controller name → Class mapping
    CONTROLLER_CLASS_MAP = {
        "freedrive_controller": FreedriveController,
        "joint_trajectory_controller": JointTrajectoryController,
        "cartesian_motion_controller": CartesianController,
        "position_controller": PositionController,
    }

    def __init__(self, node, controllers_config):
        self.node = node
        self.controllers = {}

        self.current_active_controller = None
        self.is_freeze_active = False

        self.controller_client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.controller_whitelist = [
            name for name, props in controllers_config.items() if props["whitelisted"]
        ]

        self.active_controllers = set()
        self._controller_check_thread = threading.Thread(
            target=self._monitor_active_controllers_loop,
            daemon=True
        )
        self._controller_check_thread.start()

        self.is_freeze_active = False
        self._freeze_monitor_thread = threading.Thread(
            target=self._monitor_freeze_controller_loop,
            daemon=True
        )
        self._freeze_monitor_thread.start()

        self.found_controllers_by_base = {base: [] for base in self.controller_whitelist}

    # Threaded method to monitor active controllers
    def _monitor_active_controllers_loop(self):
        """Background thread: periodically check active controllers."""
        while rclpy.ok():
            active = self._get_active_controllers_sync()
            if active is not None:
                self.active_controllers = active
            time.sleep(1.0)  # check every second (adjust as needed)

    # Synchronous method to get active controllers
    def _get_active_controllers_sync(self):
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Controller manager service not available.")
            return None

        req = ListControllers.Request()
        future = self.controller_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        try:
            response = future.result()

            found = set()
            for controller in response.controller:
                for base in self.controller_whitelist:
                    if controller.name.startswith(base) and controller.state == "active":
                        found.add(base)

            return found

        except Exception as e:
            self.node.get_logger().error(f"Error fetching controllers: {e}")
            return None

    # Method to get all controllers and their states
    def get_all_controllers(self):
        """Checks which controllers are active and updates the state machine."""
        if not self.controller_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                "Controller manager service not available.", throttle_duration_sec=10.0
            )
            return

        req = ListControllers.Request()
        future = self.controller_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        try:
            response = future.result()

            # Reset found controllers by base
            for base in self.found_controllers_by_base:
                self.found_controllers_by_base[base] = []

            # Populate found controllers by base name
            for controller in response.controller:
                for base in self.controller_whitelist:
                    if controller.name.startswith(base):
                        self.found_controllers_by_base[base].append(
                            {controller.name: controller.state}
                        )
            return self.found_controllers_by_base

        except Exception as e:
            self.node.get_logger().error(
                f"Failed to list controllers: {e}", throttle_duration_sec=10.0
            )

    # Threaded method to monitor freeze controller
    def _monitor_freeze_controller_loop(self):
        """Continuously check if the freeze controller is active."""
        while rclpy.ok():
            self.is_freeze_active = self.check_freeze_active()
            time.sleep(1.0)  # Adjust polling rate as needed
    
    # Method to check if any freeze controller is active
    def check_freeze_active(self):
        """Returns True if any freeze controller (with prefix 'freeze_controller') is active."""
        if not self.controller_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("Controller manager service not available.")
            return False

        req = ListControllers.Request()
        future = self.controller_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        try:
            response = future.result()
            for controller in response.controller:
                if controller.name.startswith("freeze_controller") and controller.state == "active":
                    return True
            return False
        except Exception as e:
            self.node.get_logger().error(f"Failed to check freeze controller: {e}")
            return False
