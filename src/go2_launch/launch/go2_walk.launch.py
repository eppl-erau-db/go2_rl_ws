from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Shared parameters
    shared_params = {'network_interface': "enp114s0"}  # TODO: CHANGE TO YOUR INTERFACE NAME

    return LaunchDescription([
        # Nodes to be launched
        Node(
            package='blind_locomotion',
            executable='controller_commands',
            name='controller_commands'
        ),
        Node(
            package='blind_locomotion',
            executable='rl_actions',
            name='rl_actions'
        ),
    ])
