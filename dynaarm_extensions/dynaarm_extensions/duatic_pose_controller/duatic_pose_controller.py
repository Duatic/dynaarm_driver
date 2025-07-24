# duatic_pose_controller_node.py

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import JointState
from ik_solver import IKSolver


class DuaticPoseController(Node):
    def __init__(self):
        super().__init__('duatic_pose_controller')

        # Initialize IK solver
        self.ik = IKSolver(
            xacro_path="/ros2_ws/src/dynaarm_demo/dynaarm_single_example/dynaarm_single_example_description/urdf/dynaarm_single_example.urdf.xacro",
            urdf_path="config/robot.urdf",
            ee_frame_name="flange"
        )

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

        # Service client to switch servo modes
        self.cmd_type_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")

        # Joint names
        self.joint_names = [
            "shoulder_rotation",
            "shoulder_flexion",
            "elbow_flexion",
            "forearm_rotation",
            "wrist_flexion",
            "wrist_rotation",
        ]

        # Loop frequency and velocity limits
        self.dt = 0.05  # 100 Hz
        self.max_joint_speed = 1.0  # rad/s
        self.threshold = 1e-3 # Threshold for joint value comparison

        # State storage
        self.current_joint_values = []
        self.latest_pose = None

        # Start periodic control loop
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        # Set servo mode to JointJog
        self.switch_servo_mode_async(0)

        self.get_logger().info("Duatic Pose Controller started at 100 Hz.")

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.old_pose = msg

    def joint_state_callback(self, msg: JointState):
        self.current_joint_values = list(msg.position)

    def switch_servo_mode_async(self, command_type: int):
        if not self.cmd_type_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("Servo command type service not available.")
            return

        switch_cmd = ServoCommandType.Request()
        switch_cmd.command_type = command_type
        self.cmd_type_client.call_async(switch_cmd)

    def control_loop(self):

        if self.latest_pose is None or not self.current_joint_values:
            return
        
        if self.latest_pose.header.stamp == self.old_pose.header.stamp:
            pose = self.latest_pose.pose
            pose_list = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]

            # self.q_sol, error_info = self.ik.solve_pose(pose_list, q_init=self.current_joint_values)
            self.q_sol, error_info = self.ik.solve_pose(pose_list)
            self.get_logger().info(f"Error info: {error_info['position_error']:.4f}")
            if self.q_sol is None or error_info["position_error"] > 1.0:
                self.get_logger().warn(f"IK solution failed or error too high: {error_info['position_error']}")
                return

        
        if all(abs(jv - cjv) < self.threshold for jv, cjv in zip(self.q_sol, self.current_joint_values)):
            self.handle_joint_jog([0.0]*len(self.joint_names), 1.0)
            return

        velocities, duration = self.joint_to_velocities(self.q_sol)
        self.handle_joint_jog(velocities, duration)

    def joint_to_velocities(self, joint_values):
        if len(self.current_joint_values) != len(joint_values):
            return [0.0] * len(joint_values), self.dt

        diffs = [jv - cjv for jv, cjv in zip(joint_values, self.current_joint_values)]

        # Damping factor to smooth movement
        damping = 0.1

        # Duration can still be based on maximum diff
        max_diff = max(abs(d) for d in diffs)
        # duration = max(max_diff / self.max_joint_speed, self.dt)
        duration = self.dt
        velocities = [damping * (d / duration) for d in diffs]

        # Cap max velocity
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
        self.get_logger().debug(f"Published joint jog: {velocities} (duration: {duration:.3f}s)")


def main(args=None):
    rclpy.init(args=args)
    node = DuaticPoseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
