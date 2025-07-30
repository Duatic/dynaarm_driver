import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply

from dynaarm_extensions.duatic_helpers.duatic_pinocchio_helper import DuaticPinocchioHelper
from dynaarm_extensions.duatic_helpers.duatic_marker_helper import DuaticMarkerHelper
from dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper

class JoyToPosePublisher(Node):
    def __init__(self):
        super().__init__("joy_to_pose_publisher")

        self.pose_pub = self.create_publisher(
            PoseStamped, "/duatic_pose_controller/target_frame", 10
        )
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.robot_helper = DuaticRobotsHelper(self)
        self.pin_helper = DuaticPinocchioHelper(self)
        self.marker_helper = DuaticMarkerHelper(self)

        self.base_frame = "base"
        self.ee_frame = "flange"
        self.current_pose = None
        self.scale = 0.005

    def joy_callback(self, msg):
        if len(self.robot_helper.get_joint_states()) <= 0:
            return

        if self.current_pose is None:
            current_joint_values = self.robot_helper.get_joint_states()            
            self.current_pose = self.pin_helper.get_fk_as_pose_stamped(current_joint_values)

        # --- Translation ---
        x = msg.axes[0]
        y = msg.axes[1]
        z = msg.axes[3]

        # --- Rotation ---
        roll = msg.axes[2]                
        pitch = float(msg.buttons[7]) - float(msg.buttons[8])        
        yaw = float(msg.axes[4] > 0.5) - float(msg.axes[5] > 0.5)

        # --- Prioritization ---        
        # Either x or y can be active, but not both and if pitch is pressed the axis are ignored
        if abs(pitch) > 1e-4:
            lx, ly = 0.0, 0.0
        elif abs(x) > abs(y) and abs(x) > 1e-4:
            lx, ly = x, 0.0
        elif abs(y) > 1e-4:
            lx, ly = 0.0, y
        else:
            lx, ly = 0.0, 0.0

        # Or either z or roll can be active, but not both
        if abs(pitch) > 1e-4:
            lz, d_roll = 0.0, 0.0
        elif abs(z) > abs(roll) and abs(z) > 1e-4:
            lz, d_roll = z, 0.0            
        elif abs(roll) > 1e-4:
            lz, d_roll = 0.0, roll
        else:
            lz, d_roll = 0.0, 0.0
        
        # Scaling
        linear_speed = 0.2
        angular_speed = 0.3

        # Update Position
        self.current_pose.pose.position.x += lx * linear_speed * self.scale
        self.current_pose.pose.position.y += ly * linear_speed * self.scale
        self.current_pose.pose.position.z += lz * linear_speed * self.scale

        # Update Orientation (Apply Incremental Rotations)
        d_roll *= angular_speed * self.scale
        pitch *= angular_speed * self.scale
        yaw *= angular_speed * self.scale

        q_roll = quaternion_from_euler(d_roll, 0, 0)
        q_pitch = quaternion_from_euler(0, pitch, 0)
        q_yaw = quaternion_from_euler(0, 0, yaw)

        current_q = self.current_pose.pose.orientation
        q_current = [current_q.x, current_q.y, current_q.z, current_q.w]
        q_new = quaternion_multiply(q_current, q_roll)
        q_new = quaternion_multiply(q_new, q_pitch)
        q_new = quaternion_multiply(q_new, q_yaw)
        norm = (q_new[0] ** 2 + q_new[1] ** 2 + q_new[2] ** 2 + q_new[3] ** 2) ** 0.5
        q_new = [q / norm for q in q_new]

        self.current_pose.pose.orientation.x = q_new[0]
        self.current_pose.pose.orientation.y = q_new[1]
        self.current_pose.pose.orientation.z = q_new[2]
        self.current_pose.pose.orientation.w = q_new[3]

        self.current_pose.header.frame_id = self.base_frame
        self.current_pose.header.stamp = self.get_clock().now().to_msg()

        self.marker_helper.create_pose_markers(self.current_pose, self.base_frame)
        self.pose_pub.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()