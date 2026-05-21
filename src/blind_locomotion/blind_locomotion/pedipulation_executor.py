#!/usr/bin/env python3
"""Timer-driven blind push pose executor for the pedipulation policy."""

import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from scipy.interpolate import CubicSpline
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    from lidar_obstacle_detection_msgs.msg import ObstacleList
except ImportError:
    ObstacleList = None

try:
    from blind_locomotion.ros_node_shutdown import spin_until_shutdown
except ModuleNotFoundError:
    from ros_node_shutdown import spin_until_shutdown


def clamp(value, lo, hi):
    """Clamp *value* into the inclusive range [lo, hi]."""
    return min(max(value, lo), hi)


def ordered_range(lo, hi):
    """Return the bounds in ascending order."""
    if lo <= hi:
        return (lo, hi)
    return (hi, lo)


class PedipulationExecutor(Node):
    """Publish a safe home pose and stream a blind push trajectory on demand."""

    HOME_HOLD = 'HOME_HOLD'
    EXECUTING = 'EXECUTING'

    def __init__(self):
        super().__init__('pedipulation_executor')

        self.declare_parameter('pose_command_topic', 'pose_command')
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('trajectory_duration_sec', 5.0)
        self.declare_parameter(
            'execute_service_name',
            'pedipulation/execute_push',
        )
        self.declare_parameter(
            'cancel_service_name',
            'pedipulation/cancel_push',
        )
        self.declare_parameter('pose_x_min', 0.20)
        self.declare_parameter('pose_x_max', 0.50)
        self.declare_parameter('pose_y_min', -0.30)
        self.declare_parameter('pose_y_max', -0.15)
        self.declare_parameter('pose_z_min', 0.0)
        self.declare_parameter('pose_z_max', 0.30)
        self.declare_parameter('push_mid_z', 0.075)
        self.declare_parameter('obstacle_list_topic', '/lidar_obstacle_detection/obstacle_list')
        self.declare_parameter('obstacle_read_timeout_sec', 1.5)

        self.pose_command_topic = str(
            self.get_parameter('pose_command_topic').value
        )
        self.publish_rate_hz = max(
            float(self.get_parameter('publish_rate_hz').value),
            1.0,
        )
        self.trajectory_duration_sec = max(
            float(self.get_parameter('trajectory_duration_sec').value),
            0.1,
        )
        self.execute_service_name = str(
            self.get_parameter('execute_service_name').value
        )
        self.cancel_service_name = str(
            self.get_parameter('cancel_service_name').value
        )
        self.pose_x_range = ordered_range(
            float(self.get_parameter('pose_x_min').value),
            float(self.get_parameter('pose_x_max').value),
        )
        self.pose_y_range = ordered_range(
            float(self.get_parameter('pose_y_min').value),
            float(self.get_parameter('pose_y_max').value),
        )
        self.pose_z_range = ordered_range(
            float(self.get_parameter('pose_z_min').value),
            float(self.get_parameter('pose_z_max').value),
        )
        self.push_mid_z = clamp(
            float(self.get_parameter('push_mid_z').value),
            self.pose_z_range[0],
            self.pose_z_range[1],
        )

        self.home_position = np.array(
            [self.pose_x_range[0], self.pose_y_range[0], self.pose_z_range[0]],
            dtype=np.float32,
        )
        self.default_push_position = np.array(
            [self.pose_x_range[1], self.pose_y_range[0], self.push_mid_z],
            dtype=np.float32,
        )
        self.push_position = self.default_push_position.copy()
        self.obstacle_list_topic = str(
            self.get_parameter('obstacle_list_topic').value
        )
        self.obstacle_read_timeout_sec = max(
            float(self.get_parameter('obstacle_read_timeout_sec').value),
            0.05,
        )
        # Persistent obstacle cache and subscriber
        self._latest_obstacle = None
        self._last_obstacle_time = 0.0
        if ObstacleList is not None:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self._obstacle_sub = self.create_subscription(
                ObstacleList,
                self.obstacle_list_topic,
                self._obstacle_callback,
                qos,
            )
        else:
            self._obstacle_sub = None

        self.position_spline = self._build_position_spline()
        self.substate = self.HOME_HOLD
        self.execution_start_time = None

        self.pose_publisher = self.create_publisher(
            Pose,
            self.pose_command_topic,
            10,
        )
        self.execute_service = self.create_service(
            Trigger,
            self.execute_service_name,
            self._handle_execute_push,
        )
        self.cancel_service = self.create_service(
            Trigger,
            self.cancel_service_name,
            self._handle_cancel_push,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._timer_callback,
        )

        self.get_logger().info(
            'Pedipulation executor started: topic=%s rate=%.1fHz duration=%.2fs'
            % (
                self.pose_command_topic,
                self.publish_rate_hz,
                self.trajectory_duration_sec,
            )
        )
        self.get_logger().info(
            'Home pose=(%.3f, %.3f, %.3f) push=(%.3f, %.3f, %.3f) '
            'quat[wxyz]=(1.000, 0.000, 0.000, 0.000)'
            % (
                self.home_position[0],
                self.home_position[1],
                self.home_position[2],
                self.push_position[0],
                self.push_position[1],
                self.push_position[2],
            )
        )

    def _obstacle_callback(self, msg):
        """Subscriber callback: cache latest obstacle message with timestamp."""
        self._latest_obstacle = msg
        self._last_obstacle_time = time.monotonic()

    def _try_update_push_position_from_obstacle(self):
        """Sample the obstacle list topic once and update push position XY if valid.

        Returns True only when exactly one valid obstacle is seen and the push
        position was updated. Otherwise returns False and leaves the executor in
        home-hold behavior.
        """
        if ObstacleList is None or self._obstacle_sub is None:
            self.get_logger().info(
                'lidar_obstacle_detection_msgs not available or subscriber not created; using default push position'
            )
            return False

        if self._latest_obstacle is None:
            self.get_logger().info(
                f'No obstacle message received on {self.obstacle_list_topic}; '
                'using default push position'
            )
            return False

        age = time.monotonic() - self._last_obstacle_time
        if age > float(self.obstacle_read_timeout_sec):
            self.get_logger().info(
                f'Latest obstacle message is stale (age={age:.3f}s); using default push position'
            )
            return False

        msg = self._latest_obstacle
        if not hasattr(msg, 'obstacles'):
            self.get_logger().info(
                'Received obstacle message does not contain obstacles field; using default push position'
            )
            return False

        if len(msg.obstacles) != 1:
            self.get_logger().info(
                f'Expected exactly 1 obstacle, but got {len(msg.obstacles)}; using default push position'
            )
            return False

        obstacle = msg.obstacles[0]
        closest_point = obstacle.closest_surface_point

        if not (math.isfinite(closest_point.x) and math.isfinite(closest_point.y)):
            self.get_logger().info(
                f'Closest surface point contains non-finite values '
                f'(x={closest_point.x}, y={closest_point.y}); using default push position'
            )
            return False

        clamped_x = clamp(float(closest_point.x), self.pose_x_range[0], self.pose_x_range[1])
        clamped_y = clamp(float(closest_point.y), self.pose_y_range[0], self.pose_y_range[1])
        self.push_position[0] = clamped_x
        self.push_position[1] = clamped_y
        self.get_logger().info(
            f'Updated push position from obstacle: x={clamped_x:.3f}, y={clamped_y:.3f}'
        )
        return True

    def _build_position_spline(self, start=None, middle=None, duration=None):
        """Build a cubic spline with an extra intermediate push-offset waypoint."""
        if start is None:
            start = self.home_position
        if middle is None:
            middle = self.push_position
        if duration is None:
            duration = self.trajectory_duration_sec

        offset_point = np.asarray(middle, dtype=np.float32) + np.array(
            [0.15, -0.15, 0.0],
            dtype=np.float32,
        )
        times = np.array(
            [0.0, 0.33 * duration, 0.66 * duration, duration],
            dtype=np.float64,
        )
        waypoints = np.vstack((start, middle, offset_point, self.home_position))
        return CubicSpline(times, waypoints, axis=0, bc_type=((1, np.zeros(3)), (1, np.zeros(3))))

    def _pose_msg_from_position(self, position):
        msg = Pose()
        msg.position.x = float(position[0])
        msg.position.y = float(position[1])
        msg.position.z = float(position[2])
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        return msg

    def _set_home_hold(self):
        self.substate = self.HOME_HOLD
        self.execution_start_time = None

    def _evaluate_trajectory(self, elapsed_sec):
        sample_time = clamp(float(elapsed_sec), 0.0, self.trajectory_duration_sec)
        current_position = np.asarray(self.position_spline(sample_time), dtype=np.float32)
        current_position[2]=min(current_position[2], 0.075)  # TODO: DONT HARDCODE THIS, but for some reason the spline is generating Z values above 0.075
        self.get_logger().info(f'current_pos: x={current_position[0]:.3f}, y={current_position[1]:.3f}, z={current_position[2]:.3f}')

        return current_position

    def _current_target_position(self):
        if self.substate != self.EXECUTING or self.execution_start_time is None:
            return self.home_position.copy()

        elapsed_sec = (
            self.get_clock().now() - self.execution_start_time
        ).nanoseconds * 1e-9

        if elapsed_sec >= self.trajectory_duration_sec:
            self._set_home_hold()
            self.get_logger().info(
                'Blind push trajectory complete; holding home pose'
            )
            return self.home_position.copy()

        return self._evaluate_trajectory(elapsed_sec)

    def _timer_callback(self):
        self.pose_publisher.publish(
            self._pose_msg_from_position(self._current_target_position())
        )

    def _handle_execute_push(self, _request, response):
        if self.substate == self.EXECUTING:
            response.success = False
            response.message = 'Push trajectory already executing'
            return response

        self.get_logger().info('I WILL GET OBSTACLE AND ADAPT SPLINE')
        if not self._try_update_push_position_from_obstacle():
            self._set_home_hold()
            response.success = False
            response.message = 'No exact single obstacle detected; staying in home hold'
            self.get_logger().info(response.message)
            return response

        self.position_spline = self._build_position_spline()
        self.substate = self.EXECUTING
        self.execution_start_time = self.get_clock().now()
        response.success = True
        response.message = 'Started blind push trajectory'
        self.get_logger().info(response.message)
        return response

    def _handle_cancel_push(self, _request, response):
        was_executing = self.substate == self.EXECUTING
        self._set_home_hold()
        response.success = True
        response.message = (
            'Cancelled active push trajectory'
            if was_executing
            else 'Already holding safe pose'
        )
        if was_executing:
            self.get_logger().info(response.message)
        return response


def main(args=None):
    """Run the pedipulation executor node."""
    rclpy.init(args=args)
    node = PedipulationExecutor()
    spin_until_shutdown(node)


if __name__ == '__main__':
    main()
