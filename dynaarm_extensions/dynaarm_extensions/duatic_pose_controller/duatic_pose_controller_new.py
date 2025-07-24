import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from duatic_ik_solver.srv import ComputeIK  # Custom IK service


class DuaticPoseController(Node):
    def __init__(self):
        super().__init__('duatic_pose_controller')

        # Create IK service client
        self.ik_client = self.create_client(ComputeIK, "/compute_ik")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for IK service...")

        # Publisher to servo interface
        self.joint_publisher = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/duatic_pose_controller/target_frame',
            self.pose_callback,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Joint names (should match URDF)
        self.joint_names = [
            "shoulder_rotation",
            "shoulder_flexion",
            "elbow_flexion",
            "forearm_rotation",
            "wrist_flexion",
            "wrist_rotation",
        ]

        # Control loop
        self.dt = 0.05  # 20 Hz
        self.max_joint_speed = 1.0  # rad/s
        self.threshold = 1e-3  # IK stability check

        self.current_joint_values = []
        self.latest_pose = None
        self.old_pose = None
        self.q_sol = None

        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Duatic Pose Controller started.")

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def joint_state_callback(self, msg: JointState):
        self.current_joint_values = list(msg.position)

    def control_loop(self):
        if self.latest_pose is None or not self.current_joint_values:
            return

        # Only solve IK if pose has changed
        if self.old_pose and self.latest_pose.header.stamp == self.old_pose.header.stamp:
            return

        # Update old_pose marker
        self.old_pose = self.latest_pose

        # Prepare request
        req = ComputeIK.Request()
        req.target_pose = self.latest_pose.pose

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.05)

        if not future.done():
            self.get_logger().warn("IK service call timed out.")
            return

        response = future.result()
        if not response.success:
            self.get_logger().warn("IK solution failed.")
            return

        self.q_sol = list(response.joint_solution.position)

        if all(abs(jv - cjv) < self.threshold for jv, cjv in zip(self.q_sol, self.current_joint_values)):
            self.handle_joint_jog([0.0] * len(self.joint_names), 1.0)
            self.get_logger().info("No significant joint movement detected, skipping jog.")
            return

        velocities, duration = self.joint_to_velocities(self.q_sol)
        self.handle_joint_jog(velocities, duration)

    def joint_to_velocities(self, joint_values):
        if len(self.current_joint_values) != len(joint_values):
            return [0.0] * len(joint_values), self.dt

        diffs = [jv - cjv for jv, cjv in zip(joint_values, self.current_joint_values)]
        damping = 0.1
        max_diff = max(abs(d) for d in diffs)
        duration = self.dt
        velocities = [damping * (d / duration) for d in diffs]
        velocities = [max(min(v, self.max_joint_speed), -self.max_joint_speed) for v in velocities]
        return velocities, duration

    def handle_joint_jog(self, velocities, duration):
        joint_jog = JointJog()
        joint_jog.header.stamp = self.get_clock().now().to_msg()
        joint_jog.joint_names = self.joint_names
        joint_jog.velocities = velocities
        joint_jog.displacements = []
        joint_jog.duration = duration
        self.joint_publisher.publish(joint_jog)
        self.get_logger().debug(f"Published joint jog: {velocities}")


def main(args=None):
    rclpy.init(args=args)
    node = DuaticPoseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
