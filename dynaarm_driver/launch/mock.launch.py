# Author: Timo Schwarzer
# Date: December 22, 2023
# Description: Launch a Mitsubishi RV-2FR robot URDF file using Rviz.
# http://www.robolynk.de

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    start_rviz = LaunchConfiguration("start_rviz")
    dof = LaunchConfiguration("dof")
    covers = LaunchConfiguration("covers")

    dof_value = dof.perform(context)
    covers_value = covers.perform(context)
   
    # Robot description package
    pkg_share_description = FindPackageShare(package="dynaarm_description").find("dynaarm_description")
    doc = xacro.parse(open(os.path.join(pkg_share_description, "xacro/dynaarm_standalone.urdf.xacro")))
    xacro.process_doc(doc, mappings={'dof': dof_value, 
                                    'covers': covers_value,
                                    'mode': 'mock'})    
    robot_description = {"robot_description": doc.toxml()}

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Launch RViz    
    rviz_config_file = PathJoinSubstitution([pkg_share_description, "launch/config.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node,
            on_exit=[rviz_node],
        )
    )

    # Real Hardware
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dynaarm_driver"),
            "config",
            "controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    startup_controller_name = "joint_trajectory_controller" 

    startup_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[startup_controller_name, "-c", "/controller_manager"],
    )

    # The controller to start variable
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node, on_exit=[startup_controller_node]
        )
    )

    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="dof",
            default_value="6dof",
            description="Select the desired degrees of freedom (dof)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="covers",
            default_value="false",
            description="Show or hide the covers of the robot",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
