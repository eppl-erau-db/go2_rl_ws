from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Shared parameters
    shared_params = {'network_interface': "enp114s0"}  # TODO: CHANGE TO YOUR INTERFACE NAME

    return LaunchDescription([
        # Nodes to be launched
        Node(
            package='go2_sdk_integration',
            executable='go2_motion_shut_off',
            name='go2_motion_shut_off',
            parameters=[shared_params],
        ),
        Node(
            package='go2_sdk_integration',
            executable='go2_audio_shut_off',
            name='go2_audio_shut_off',
            parameters=[shared_params],
        ),
        Node(
            package='unitree_ros2_python',
            executable='go2_base_ang_vel',
            name='go2_base_ang_vel'
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
            executable='go2_rl_actions_onnx',
            name='go2_rl_actions_onnx'
        ),
    ])
