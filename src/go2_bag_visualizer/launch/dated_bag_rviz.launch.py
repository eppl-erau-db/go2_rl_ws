#!/usr/bin/env python3
"""Launch RViz, marker generation, and playback for dated Go2 rosbag2 files."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from go2_bag_visualizer.bag_finder import (
    DEFAULT_DATE_SUBSTRING,
    choose_bag,
    find_dated_bags,
)


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _topic_counts(bag_path: Path) -> dict[str, int]:
    metadata = yaml.safe_load((bag_path / 'metadata.yaml').read_text())
    topics = metadata['rosbag2_bagfile_information']['topics_with_message_count']
    return {
        entry['topic_metadata']['name']: int(entry['message_count'])
        for entry in topics
    }


def _present_topics(topic_counts: dict[str, int], requested_topics: list[str]) -> list[str]:
    topics = []
    for topic in requested_topics:
        if topic and topic in topic_counts and topic not in topics:
            topics.append(topic)
    return topics


def _launch_setup(context, *_args, **_kwargs):
    package_share = Path(get_package_share_directory('go2_bag_visualizer'))

    bag_path_arg = LaunchConfiguration('bag_path').perform(context).strip()
    bag_root = Path(LaunchConfiguration('bag_root').perform(context)).expanduser()
    date = LaunchConfiguration('date').perform(context)
    recursive = _as_bool(LaunchConfiguration('recursive').perform(context))
    bag_index = int(LaunchConfiguration('bag_index').perform(context))

    if bag_path_arg:
        bag_path = Path(bag_path_arg).expanduser().resolve()
        matches = []
    else:
        matches = find_dated_bags(bag_root, date, recursive)
        bag_path = choose_bag(bag_root, date, bag_index, recursive)

    if not (bag_path / 'metadata.yaml').is_file():
        raise FileNotFoundError(f'{bag_path} does not look like a rosbag2 directory')

    topic_counts = _topic_counts(bag_path)

    obstacle_topic = LaunchConfiguration('obstacle_topic').perform(context)
    obstacle_marker_topic = LaunchConfiguration('obstacle_marker_topic').perform(context)
    goal_pose_topic = LaunchConfiguration('goal_pose_topic').perform(context)
    nav_goal_topic = LaunchConfiguration('nav_goal_topic').perform(context)
    nav_goal_marker_topic = LaunchConfiguration('nav_goal_marker_topic').perform(context)
    pose_command_topic = LaunchConfiguration('pose_command_topic').perform(context)
    show_pose_command_markers = _as_bool(
        LaunchConfiguration('show_pose_command_markers').perform(context)
    )
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    tf_topic = LaunchConfiguration('tf_topic').perform(context)
    tf_static_topic = LaunchConfiguration('tf_static_topic').perform(context)
    marker_topic = LaunchConfiguration('marker_topic').perform(context)
    play_rate = LaunchConfiguration('play_rate').perform(context)
    clock_hz = LaunchConfiguration('clock_hz').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if not rviz_config:
        rviz_config = str(package_share / 'rviz' / 'go2_bag_visualizer.rviz')

    play_cmd = [
        'ros2',
        'bag',
        'play',
        str(bag_path),
        '--clock',
        clock_hz,
        '--rate',
        play_rate,
        '--delay',
        '1.0',
    ]
    if _as_bool(LaunchConfiguration('loop').perform(context)):
        play_cmd.append('--loop')
    if _as_bool(LaunchConfiguration('start_paused').perform(context)):
        play_cmd.append('--start-paused')
    if not _as_bool(LaunchConfiguration('play_all_topics').perform(context)):
        topics_to_play = _present_topics(
            topic_counts,
            [
                tf_topic,
                tf_static_topic,
                odom_topic,
                obstacle_topic,
                obstacle_marker_topic,
                goal_pose_topic,
                nav_goal_topic,
                nav_goal_marker_topic,
            ],
        )
        if show_pose_command_markers:
            topics_to_play = _present_topics(
                topic_counts,
                [*topics_to_play, pose_command_topic],
            )
        if topics_to_play:
            play_cmd.extend(['--topics', *topics_to_play])

    goal_count = topic_counts.get(goal_pose_topic)
    nav_goal_count = topic_counts.get(nav_goal_topic)
    pose_command_count = topic_counts.get(pose_command_topic)
    obstacle_marker_count = topic_counts.get(obstacle_marker_topic)
    nav_goal_marker_count = topic_counts.get(nav_goal_marker_topic)

    actions = [
        LogInfo(
            msg=(
                f'[go2_bag_visualizer] Selected bag: {bag_path} '
                f'({len(matches)} auto-discovered matches)'
            )
        ),
        LogInfo(
            msg=(
                f'[go2_bag_visualizer] {goal_pose_topic} messages: '
                f'{0 if goal_count is None else goal_count}; '
                f'{nav_goal_topic} messages: '
                f'{0 if nav_goal_count is None else nav_goal_count}; '
                f'{pose_command_topic} messages: '
                f'{0 if pose_command_count is None else pose_command_count} '
                '(end-effector command, not nav goal)'
            )
        ),
        LogInfo(
            msg=(
                f'[go2_bag_visualizer] Recorded marker arrays: '
                f'{obstacle_marker_topic}='
                f'{0 if obstacle_marker_count is None else obstacle_marker_count}, '
                f'{nav_goal_marker_topic}='
                f'{0 if nav_goal_marker_count is None else nav_goal_marker_count}'
            )
        ),
        Node(
            package='go2_bag_visualizer',
            executable='marker_node',
            name='go2_bag_marker_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                    'obstacle_topic': obstacle_topic,
                    'goal_pose_topic': goal_pose_topic,
                    'pose_command_topic': pose_command_topic,
                    'show_pose_command_markers': show_pose_command_markers,
                    'marker_topic': marker_topic,
                    'fallback_frame_id': LaunchConfiguration('fallback_frame_id'),
                    'pose_command_frame_id': LaunchConfiguration(
                        'pose_command_frame_id'
                    ),
                    'obstacle_lifetime_sec': ParameterValue(
                        LaunchConfiguration('obstacle_lifetime_sec'),
                        value_type=float,
                    ),
                    'goal_lifetime_sec': ParameterValue(
                        LaunchConfiguration('goal_lifetime_sec'),
                        value_type=float,
                    ),
                    'show_obstacle_labels': ParameterValue(
                        LaunchConfiguration('show_obstacle_labels'),
                        value_type=bool,
                    ),
                    'show_closest_surface_points': ParameterValue(
                        LaunchConfiguration('show_closest_surface_points'),
                        value_type=bool,
                    ),
                }
            ],
        ),
        ExecuteProcess(cmd=play_cmd, name='dated_rosbag_play', output='screen'),
    ]

    if _as_bool(LaunchConfiguration('rviz').perform(context)):
        actions.append(
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config],
                name='rviz2',
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'bag_root',
                default_value=str(Path.home()),
                description='Directory to search for dated rosbag2 folders.',
            ),
            DeclareLaunchArgument(
                'date',
                default_value=DEFAULT_DATE_SUBSTRING,
                description='Date substring that must appear in the bag folder name.',
            ),
            DeclareLaunchArgument(
                'bag_path',
                default_value='',
                description='Explicit rosbag2 directory. Overrides bag_root/date discovery.',
            ),
            DeclareLaunchArgument(
                'bag_index',
                default_value='-1',
                description='Index into sorted matches; -1 selects the newest lexical match.',
            ),
            DeclareLaunchArgument(
                'recursive',
                default_value='false',
                description='Search bag_root recursively instead of only direct children.',
            ),
            DeclareLaunchArgument('play_rate', default_value='1.0'),
            DeclareLaunchArgument('clock_hz', default_value='30'),
            DeclareLaunchArgument('loop', default_value='false'),
            DeclareLaunchArgument('start_paused', default_value='false'),
            DeclareLaunchArgument(
                'play_all_topics',
                default_value='false',
                description='Replay every recorded topic instead of only visualization inputs.',
            ),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument(
                'rviz_config',
                default_value='',
                description='RViz config path. Empty uses this package default.',
            ),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'obstacle_topic',
                default_value='/lidar_obstacle_detection/obstacle_list',
            ),
            DeclareLaunchArgument(
                'obstacle_marker_topic',
                default_value='/lidar_obstacle_detection/obstacle_markers',
                description='Recorded obstacle MarkerArray topic to replay for RViz.',
            ),
            DeclareLaunchArgument('goal_pose_topic', default_value='/goal_pose'),
            DeclareLaunchArgument('nav_goal_topic', default_value='/nav_goal'),
            DeclareLaunchArgument(
                'nav_goal_marker_topic',
                default_value='/nav_goal_debug_markers',
                description='Recorded nav-goal MarkerArray topic to replay for RViz.',
            ),
            DeclareLaunchArgument('pose_command_topic', default_value='/pose_command'),
            DeclareLaunchArgument(
                'show_pose_command_markers',
                default_value='false',
                description=(
                    'Visualize /pose_command as an end-effector target marker. '
                    'It is not treated as a nav goal.'
                ),
            ),
            DeclareLaunchArgument('odom_topic', default_value='/odom'),
            DeclareLaunchArgument('tf_topic', default_value='/tf'),
            DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
            DeclareLaunchArgument(
                'marker_topic',
                default_value='/go2_bag_visualization/markers',
            ),
            DeclareLaunchArgument('fallback_frame_id', default_value='base_link'),
            DeclareLaunchArgument('pose_command_frame_id', default_value='base_link'),
            DeclareLaunchArgument('obstacle_lifetime_sec', default_value='0.35'),
            DeclareLaunchArgument('goal_lifetime_sec', default_value='0.0'),
            DeclareLaunchArgument('show_obstacle_labels', default_value='true'),
            DeclareLaunchArgument(
                'show_closest_surface_points',
                default_value='true',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
