import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)

    if not os.path.exists(absolute_file_path):
        raise FileNotFoundError(
            "Request file: " + absolute_file_path + " not found")

    return Path(absolute_file_path)

def generate_launch_description():
    ld = LaunchDescription()
    declare_use__mock_hardware = DeclareLaunchArgument(name='use_mock_hardware', default_value="True",
                                                         description='Use simulated mock hardware')
    ld.add_action(declare_use__mock_hardware)

    urdf_description_file = os.path.join(get_package_share_directory(
        "dynaarm_urdf"), "urdf/dynaarm_standalone.urdf.xacro")
    urdf = Command(['xacro ', urdf_description_file, " use_mock_hardware:=", LaunchConfiguration("use_mock_hardware")])
    robot_description_parameter = {
        "robot_description":  ParameterValue(urdf, value_type=str)}
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_parameter])

    rviz_config_file_path = os.path.join(get_package_share_directory("dynaarm_urdf"),'rviz/config.rviz')
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_dynaarm',
        output='both',
        emulate_tty=True,
        arguments=['-d', rviz_config_file_path],
    )


        # Obtain the controller config file for the ros2 control node
    controller_config_file = get_package_file(
        "dynaarm_driver", "config/controllers.yaml")
    # Create start command for the ros2 control node. It uses the urdf file + the controller config file to determine
    # What to start and which controllers it knows
    start_control_node_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ controller_config_file,robot_description_parameter],
        output="both",
        emulate_tty=True,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller",
                   "--controller-manager", "/controller_manager"],
    )



    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_control_node_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)
    return ld