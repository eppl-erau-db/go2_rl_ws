# Go2 Bag Visualizer

This package discovers dated Go2 rosbag2 directories, replays one bag, and
publishes RViz markers for:

- `/lidar_obstacle_detection/obstacle_list` as obstacle boxes and closest points
- `/goal_pose` as goal arrows when present
- recorded `/nav_goal_debug_markers` and
  `/lidar_obstacle_detection/obstacle_markers` MarkerArray topics

By default, discovery only includes rosbag2 directories whose names contain
`5_11`, such as `rosbag2_2026_05_11-...`. Matching bags may declare
`/goal_pose` even when they have zero `/goal_pose` messages. The May 11 bags
also record `/tf`, `/tf_static`, `/odom`, `/nav_goal`,
`/nav_goal_debug_markers`, and `/lidar_obstacle_detection/obstacle_markers`.
`/nav_goal` is a `Float32MultiArray`, so RViz shows its matching debug marker
topic instead. `/pose_command` is an end-effector command stream, so it is not
shown as a nav goal. You can enable a separate end-effector marker for it with
`show_pose_command_markers:=true`.

Default launch:

```bash
source ~/workspaces/go2_rl_ws/install/setup.bash
ros2 launch go2_bag_visualizer dated_bag_rviz.launch.py
```

Useful overrides:

```bash
ros2 launch go2_bag_visualizer dated_bag_rviz.launch.py bag_index:=0
ros2 launch go2_bag_visualizer dated_bag_rviz.launch.py bag_path:=/home/srge/rosbag2_2026_05_11-16_59_48
ros2 launch go2_bag_visualizer dated_bag_rviz.launch.py loop:=true play_rate:=0.5
ros2 launch go2_bag_visualizer dated_bag_rviz.launch.py show_pose_command_markers:=true
```

Offline matplotlib plots:

```bash
ros2 run go2_bag_visualizer plot_bags_3d
ros2 run go2_bag_visualizer plot_bags_3d --save-dir /tmp/go2_bag_plots --no-show
ros2 run go2_bag_visualizer plot_bags_3d --bag /home/srge/rosbag2_2026_05_11-16_59_48
ros2 run go2_bag_visualizer plot_bags_3d --metrics-only --no-show
```

The matplotlib viewer creates two figures per bag:

- `*_navigation_before_pose_command.png`: `/odom` before the first
  `/pose_command`, `/nav_goal_debug_markers`, and object center/contact points
  from `/lidar_obstacle_detection/obstacle_list`, plus arrows for the final
  robot heading and final nav-goal heading before the end-effector phase.
- `*_end_effector_object_force.png`: `/pose_command`, object center/contact
  motion during the end-effector phase, and `/lowstate` `foot_force` traces.
  The end-effector and object views are shown in the robot `base_link` frame.

For the navigation figure, obstacle points are transformed into `odom` using
the nearest `/odom` pose. For the end-effector figure, obstacle and
`/pose_command` points stay in `base_link`. `/nav_goal` is recorded as a
`Float32MultiArray`, so the plotter uses `/nav_goal_debug_markers` for the goal
location and heading when that topic is present.

Useful offline options:

```bash
ros2 run go2_bag_visualizer plot_bags_3d --lowstate-stride 1
ros2 run go2_bag_visualizer plot_bags_3d --pose-window-padding-sec 1.0
ros2 run go2_bag_visualizer plot_bags_3d --no-lowstate-plots
ros2 run go2_bag_visualizer plot_bags_3d --nav-goal-stride 1
ros2 run go2_bag_visualizer plot_bags_3d --metrics-csv /tmp/go2_metrics.csv --metrics-md /tmp/go2_metrics.md
```

Offline experiment metrics:

```bash
ros2 run go2_bag_visualizer plot_bags_3d --metrics-only --no-show
ros2 run go2_bag_visualizer plot_bags_3d --metrics-only --no-show --touch-force-threshold 25.0 --ee-ground-clearance 0.02
```

The metrics table is printed once per run for every valid bag. It includes the
closest navigation distance to the latest recorded goal, yaw error at that
closest point, final navigation distance, navigation path length, detected
front-right-foot object touch, maximum FR force while `/pose_command` is above
the ground clearance, touch duration, object displacement in `odom`, minimum
commanded end-effector distance to the detected object surface, best-effort
`/pose_command` checkpoint reach metrics from FR-leg FK, environment slope from
obstacle surface normals, and the experiment initial odom pose.

When a bag contains multiple reach movements, the metrics first split the
recorded `/pose_command` stream into separate active reach segments and select
the segment with the largest front-right foot-force peak. The checkpoint reach
metrics then take four recorded `/pose_command` positions at equal time
intervals through that selected segment. The commands are used as recorded,
without z caps, obstacle reconstruction, or training-domain clamps, and compared
spatially against the calibrated real FR end-effector trace from the same
segment. These distances are nearest-neighbor distances inside that reach, so
the checkpoint and real end-effector samples do not need to happen at the same
timestamp.

Object displacement is also measured over the selected reach segment, using the
object positions nearest the segment start and end in `odom`.

Touch detection uses `/lowstate.foot_force[0]` for the front-right foot and
only evaluates samples where the nearest `/pose_command.position.z` is greater
than `--ee-ground-clearance`. The default touch threshold is `25 N`; adjust
`--touch-force-threshold` if the force scale/noise changes.
