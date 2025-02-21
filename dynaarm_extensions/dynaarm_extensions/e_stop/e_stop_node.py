import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import Joy

NANOSECONDS_TO_SECONDS = 1_000_000_000

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('dynaarm_emergency_stop_node')

        # Declare and fetch the emergency stop button parameter
        self.declare_parameter('emergency_stop_button', 8)
        self.emergency_stop_button = int(self.get_parameter('emergency_stop_button').value)

        # Track gamepad state and emergency stop status
        self.gamepad_connected = False
        self.show_gamepad_connected_warning = True
        self.emergency_stop_active = True        
        self.e_stop_button_release = True
        self.e_stop_pressed_state = False
        self.emergency_stop_pressed_time = None
        self.last_joy_received_time = self.get_clock().now()
        self.last_toggle_time = self.get_clock().now()  
        
        # Subscribe to Joy topic
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Create a service client for switching controllers
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # Timer to monitor gamepad and emergency stop state at 500 Hz (0.002s interval)
        self.control_loop_timer = self.create_timer(0.002, self.control_loop)

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
        if not self.gamepad_connected and self.show_gamepad_connected_warning:
            self.get_logger().warning("Waiting for gamepad to connect.")
            self.show_gamepad_connected_warning = False

        # Gamepad disconnects, but allow a grace period before assuming disconnection
        if self.gamepad_connected and (now - self.last_joy_received_time) > Duration(seconds=1.0):
            if (now - self.last_toggle_time).nanoseconds / NANOSECONDS_TO_SECONDS >= 2:
                self.get_logger().warn("Gamepad disconnected! Activating freeze_controller.")
                self.show_gamepad_connected_warning = True
                self.set_emergency_stop_state(True)  
                self.gamepad_connected = False

        # Handle emergency stop logic
        if self.emergency_stop_active:
            if self.e_stop_pressed_state:
                if self.emergency_stop_pressed_time is None:
                    self.emergency_stop_pressed_time = now

                if (now - self.emergency_stop_pressed_time).nanoseconds / 1e9 >= 3:
                    self.get_logger().info("Emergency stop released. Resuming control.")
                    self.set_emergency_stop_state(False)  
                    self.e_stop_button_release = False
                    self.emergency_stop_pressed_time = None
            else:
                self.emergency_stop_pressed_time = None  
        else:
            if not self.e_stop_pressed_state:
                self.e_stop_button_release = True

            if self.e_stop_pressed_state and self.e_stop_button_release:
                self.get_logger().error("EMERGENCY STOP ACTIVATED!")
                self.set_emergency_stop_state(True)  

    def set_emergency_stop_state(self, activate: bool):
        """Sets emergency stop state and triggers controller switch only if state changes."""
        if activate == self.emergency_stop_active:
            return  

        previous_state = self.emergency_stop_active  # Store previous state in case of failure
        self.emergency_stop_active = activate
        self.last_toggle_time = self.get_clock().now()  
        self.toggle_freeze_controller(previous_state)

    def toggle_freeze_controller(self, previous_state: bool):
        """Activates or deactivates the freeze_controller asynchronously with rollback on failure."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Controller Manager service not available. Reverting e-stop state.")
            self.emergency_stop_active = previous_state  # Revert to previous state
            return  

        request = SwitchController.Request()
        request.activate_controllers = ["freeze_controller"] if self.emergency_stop_active else []
        request.deactivate_controllers = [] if self.emergency_stop_active else ["freeze_controller"]
        request.strictness = SwitchController.Request.BEST_EFFORT

        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(lambda future: self.handle_switch_response(future, previous_state))

    def handle_switch_response(self, future, previous_state):
        """Handles the response from the controller switch request, reverting state on failure."""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f"Freeze Controller {'Activated' if self.emergency_stop_active else 'Deactivated'}")                
            else:
                self.get_logger().error("Failed to switch controllers. Reverting e-stop state.")
                self.emergency_stop_active = previous_state  # Revert to previous state
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}. Reverting e-stop state.")
            self.emergency_stop_active = previous_state  # Revert to previous state

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
