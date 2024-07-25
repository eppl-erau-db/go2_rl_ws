import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # Nodes to be launched
        Node(
            package='unitree_ros2_python',
            executable='go2_base_vel',
            name='go2_base_vel'
        ),
        Node(
            package='unitree_ros2_python',
            executable='go2_joint_pos_vel',
            name='go2_joint_pos_vel'
        ),
        Node(
            package='unitree_ros2_python',
            executable='go2_projected_gravity',
            name='go2_projected_gravity'
        ),
        Node(
            package='unitree_ros2_python',
            executable='go2_controller_commands',
            name='go2_velocity_commands'
        ),
        Node(
            package='unitree_ros2_python',
            executable='go2_rl_actions',
            name='go2_rl_actions'
        ),
    ])
