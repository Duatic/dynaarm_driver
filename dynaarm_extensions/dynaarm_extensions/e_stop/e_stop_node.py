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
from rclpy.duration import Duration
from controller_manager_msgs.srv import SwitchController, ListControllers
from sensor_msgs.msg import Joy


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__("dynaarm_emergency_stop_node")

        # Declare and fetch the emergency stop button parameter
        self.declare_parameter("emergency_stop_button", 9)
        self.emergency_stop_button = int(self.get_parameter("emergency_stop_button").value)

        # Track gamepad state and emergency stop status
        self.gamepad_connected = False
        self.show_gamepad_connected_warning = True
        self.e_stop_button_release = True
        self.e_stop_pressed_state = False
        self.was_freeze_controller_active_before = False
        self.emergency_stop_pressed_time = None
        self.last_joy_received_time = self.get_clock().now()
        self.last_toggle_time = self.get_clock().now()

        # Subscribe to Joy topic
        self.joy_subscriber = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Create service clients
        self.switch_controller_client = self.create_client(SwitchController, "controller_manager/switch_controller")
        self.list_controllers_client = self.create_client(ListControllers, "controller_manager/list_controllers")

        # Timer to monitor gamepad and emergency stop state at 100 Hz (0.01s interval)
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)

    def joy_callback(self, msg: Joy):
        """Only updates the e-stop button state and last received time."""
        self.last_joy_received_time = self.get_clock().now()

        if not self.gamepad_connected:
            self.get_logger().info("Gamepad connected.")
            self.gamepad_connected = True

        if self.emergency_stop_button is None or self.emergency_stop_button >= len(msg.buttons):
            return

        self.e_stop_pressed_state = bool(msg.buttons[self.emergency_stop_button])

    def control_loop(self):
        """Runs at 500 Hz and handles everything: e-stop, timeouts, and controller switching."""
        now = self.get_clock().now()

        # No gamepad connected
        if not self.gamepad_connected:
            self.get_logger().warning("Waiting for gamepad to connect.", throttle_duration_sec=15)

        # Gamepad disconnects, but allow a grace period before assuming disconnection
        if self.gamepad_connected and (now - self.last_joy_received_time) > Duration(seconds=1.0):
            if (now - self.last_toggle_time).nanoseconds / 10**9 >= 2:
                self.get_logger().warn("Gamepad disconnected! Activating freeze_controller.")
                self.set_freeze_controller(True)
                self.gamepad_connected = False

        # Handle emergency stop activation
        if self.e_stop_pressed_state and self.e_stop_button_release:
            self.get_logger().error("EMERGENCY STOP ACTIVATED!")
            self.set_freeze_controller(True)
            self.e_stop_button_release = False
            self.emergency_stop_pressed_time = now  # Start tracking press duration

        # Handle emergency stop deactivation (button held for 3 sec)
        elif self.e_stop_pressed_state and self.emergency_stop_pressed_time:
            hold_duration = (now - self.emergency_stop_pressed_time).nanoseconds / 1e9
            if hold_duration >= 3.0:
                if self.was_freeze_controller_active_before:
                    self.get_logger().info("Emergency stop button held for 3 sec - Deactivating freeze_controller.")
                    self.set_freeze_controller(False)
                self.emergency_stop_pressed_time = None  # Reset timer after release

        # Reset release state when button is fully released
        elif not self.e_stop_pressed_state and not self.e_stop_button_release:
            self.e_stop_button_release = True
            self.emergency_stop_pressed_time = None  # Reset timer
    
    def get_freeze_controller_state(self):
        """Queries the controller manager to check if freeze_controller is currently active."""
        if not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Controller Manager service not available.")
            return False  # Assume inactive if we can't query

        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            for controller in future.result().controller:
                if controller.name == "freeze_controller" and controller.state == "active":
                    return True
        return False

    def set_freeze_controller(self, active: bool):
        """Activates or deactivates the freeze_controller asynchronously with rollback on failure."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                "Controller Manager service not available. Reverting e-stop state."
            )
            return

        request = SwitchController.Request()
        request.activate_controllers = ["freeze_controller"] if active else []
        request.deactivate_controllers = [] if active else ["freeze_controller"]
        request.strictness = SwitchController.Request.STRICT

        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(lambda future: self.handle_switch_response(future, active))

    def handle_switch_response(self, future, activate):
        """Handles the response from the switch_controller request."""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f"Freeze Controller {'Activated' if activate else 'Deactivated'}")
                self.was_freeze_controller_active_before = not activate
            else:
                self.get_logger().warn(f"Freeze Controller switch failed. Checking actual state...")
                self.was_freeze_controller_active_before = self.get_freeze_controller_state()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}. Checking actual state...")
            self.was_freeze_controller_active_before = self.get_freeze_controller_state()

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
