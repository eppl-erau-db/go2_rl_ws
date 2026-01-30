from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Test launch file for stand/sit transitions WITHOUT the RL policy.
    
    This allows testing the go2_controller_node state machine in isolation:
    - fake_lowstate_publisher: Simulates robot state (starts laying down)
    - test_action_publisher: Publishes standing position actions at 25Hz
    - keyboard_buttons: Keyboard input for button control
    - go2_controller_node: The component under test
    - lowcmd_decoder: (optional) Human-readable /lowcmd display
    
    Usage:
        ros2 launch go2_launch test_stand_sit.launch.py
        ros2 launch go2_launch test_stand_sit.launch.py show_decoder:=true verbose:=true
    
    Keyboard controls (in keyboard_buttons terminal):
        u = UP (stand)
        d = DOWN (sit)
        s = START (start walking)
        x = SELECT (stop)
        a = A (damping/soft abort)
        b = B (kill/emergency stop)
        q = quit
    """
    
    # Launch arguments
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Enable verbose logging in go2_controller_node'
    )
    
    show_decoder_arg = DeclareLaunchArgument(
        'show_decoder',
        default_value='false',
        description='Show lowcmd_decoder output (human-readable commands)'
    )
    
    action_frequency_arg = DeclareLaunchArgument(
        'action_frequency',
        default_value='25.0',
        description='Frequency of action publishing in Hz'
    )
    
    lowstate_frequency_arg = DeclareLaunchArgument(
        'lowstate_frequency',
        default_value='500.0',
        description='Frequency of fake lowstate publishing in Hz'
    )

    return LaunchDescription([
        # Launch arguments
        verbose_arg,
        show_decoder_arg,
        action_frequency_arg,
        lowstate_frequency_arg,
        
        # Fake LowState publisher - simulates robot state starting from laying down
        # (Go2 must be laid down to disable sport mode before our controller takes over)
        Node(
            package='blind_locomotion',
            executable='fake_lowstate_publisher.py',
            name='fake_lowstate_publisher',
            parameters=[{
                'publish_frequency': LaunchConfiguration('lowstate_frequency'),
            }],
            output='screen'
        ),
        
        # Test action publisher - publishes standing position (no RL inference)
        Node(
            package='blind_locomotion',
            executable='test_action_publisher.py',
            name='test_action_publisher',
            parameters=[{
                'publish_frequency': LaunchConfiguration('action_frequency'),
            }],
            output='screen'
        ),
        
        # Keyboard button input - runs in its own terminal for keyboard capture
        # Note: This needs to run in a terminal that can capture keyboard input
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'blind_locomotion', 'keyboard_buttons.py'],
            name='keyboard_buttons',
            output='screen'
        ),
        
        # The component under test - go2_controller_node
        Node(
            package='rl_deploy',
            executable='go2_controller_node',
            name='go2_controller_node',
            parameters=[{
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen'
        ),
        
        # Optional: LowCmd decoder for human-readable output
        Node(
            package='blind_locomotion',
            executable='lowcmd_decoder.py',
            name='lowcmd_decoder',
            condition=IfCondition(LaunchConfiguration('show_decoder')),
            output='screen'
        ),
    ])
