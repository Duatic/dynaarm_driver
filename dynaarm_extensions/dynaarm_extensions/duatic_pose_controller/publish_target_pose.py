#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from ik_solver import IKSolver


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')

         # Initialize IK solver
        self.ik = IKSolver(
            xacro_path="/ros2_ws/src/dynaarm_demo/dynaarm_single_example/dynaarm_single_example_description/urdf/dynaarm_single_example.urdf.xacro",
            urdf_path="config/robot.urdf",
            ee_frame_name="flange"
        )
        
        # Publisher for Cartesian motion
        self.pose_publisher_ = self.create_publisher(PoseStamped, '/duatic_pose_controller/target_frame', 10)
        
        # Publisher for joint trajectory
        self.joint_trajectory_publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.current_joint_values = []

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer to call both publishers
        #self.timer = self.create_timer(2.0, self.timer_callback)
        

    def joint_state_callback(self, msg: JointState):
        self.current_joint_values = list(msg.position)
        self.publish_pose()

    def timer_callback(self):  # constraint projection sample
        self.publish_pose()
        #self.publish_joint_trajectory()

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'

        # Example position and orientation
        # msg.pose.position.x = 0.3
        # msg.pose.position.y = -0.3
        # msg.pose.position.z = 0.5

        # msg.pose.orientation.x = 0.0
        # msg.pose.orientation.y = 0.0
        # msg.pose.orientation.z = 1.0
        # msg.pose.orientation.w = 0.0

        self.get_logger().info(str(self.current_joint_values))

        pose = self.current_joint_values
        pose_list = [
            pose[0],
            pose[1],
            pose[2],
            pose[3],
            pose[4],
            pose[5]
        ]

        current_position = self.ik.forward_kinematics(pose_list)

        msg.pose.position.x = current_position[0] + 0.0
        msg.pose.position.y = current_position[1] + 0.0
        msg.pose.position.z = current_position[2] - 0.2
        msg.pose.orientation.x = current_position[3] + 0.0
        msg.pose.orientation.y = current_position[4] + 0.0
        msg.pose.orientation.z = current_position[5] + 0.0
        msg.pose.orientation.w = current_position[6] + 0.0

        self.get_logger().info('Publishing target pose.')
        self.pose_publisher_.publish(msg)

    def publish_joint_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.joint_names = ([
            'shoulder_rotation',
            'shoulder_flexion',
            'elbow_flexion',
            'forearm_rotation',
            'wrist_flexion',
            'wrist_rotation'
        ])

        # Set desired joint values
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 1.57, 0.0, 0.0, 0.0]  # Replace with your desired joint values
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=2)  # Reach the position in 2 seconds

        msg.points.append(point)

        self.get_logger().info('Publishing joint trajectory command.')
        self.joint_trajectory_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPosePublisher()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
