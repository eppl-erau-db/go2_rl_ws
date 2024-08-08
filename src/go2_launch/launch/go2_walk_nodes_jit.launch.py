from launch import LaunchDescription
from launch_ros.actions import Node


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
            name='go2_controller_commands'
        ),
        Node(
            package='unitree_ros2_python',
            executable='go2_rl_actions_jit',
            name='go2_rl_actions_jit'
        ),
    ])