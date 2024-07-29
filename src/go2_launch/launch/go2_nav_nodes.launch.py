from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Nodes to be launched
        Node(
            package='unitree_ros2_python',
            executable='go2_projected_gravity',
            name='go2_projected_gravity'
        ),
        Node(
            package='rl_navigation',
            executable='go2_pose_command',
            name='go2_pose_command'
        ),

        Node(
            package='rl_navigation',
            executable='go2_rl_nav_actions',
            name='go2_rl_nav_actions'
        ),
    ])
