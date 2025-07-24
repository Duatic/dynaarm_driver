#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time
import math

class ServoTwistPublisher(Node):
    def __init__(self):
        super().__init__('servo_twist_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)  # 10 Hz

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'  # Must match your robot's planning frame

        # Set linear velocity (e.g., move 0.1 m/s in +X direction)
        msg.twist.linear.x = 0.1
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        # Set angular velocity (e.g., rotate 0.1 rad/s around Z axis)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.1

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing TwistStamped command')

def main(args=None):
    rclpy.init(args=args)
    node = ServoTwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
