#!/usr/bin/env python3
"""RL navigation policy node that publishes high-level cmd_vel commands."""
import os

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from blind_locomotion.msg import Button
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState

try:
    from blind_locomotion.ros_node_shutdown import spin_until_shutdown
except ModuleNotFoundError:
    from ros_node_shutdown import spin_until_shutdown


class RLNavCommandsNode(Node):
    def __init__(self):
        super().__init__('rl_nav_commands')

        self.declare_parameter('policy_name', 'nav_policy')
        self.declare_parameter('policy_frequency', 50)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('lowstate_timeout_sec', 0.5)
        self.declare_parameter('goal_topic', 'nav_goal')
        self.declare_parameter('goal_timeout_sec', 0.5)
        self.declare_parameter('require_start_button', True)
        self.declare_parameter('default_goal', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('cmd_vel_x_limit', 1.0)
        self.declare_parameter('cmd_vel_y_limit', 1.0)
        self.declare_parameter('cmd_vel_yaw_limit', 1.0)
        self.declare_parameter('nav_output_scale', 0.25)

        policy_name = str(self.get_parameter('policy_name').value)
        policy_frequency = int(self.get_parameter('policy_frequency').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.lowstate_timeout_sec = float(self.get_parameter('lowstate_timeout_sec').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.require_start_button = bool(
            self.get_parameter('require_start_button').value
        )
        self.default_goal = self._coerce_goal_vector(
            self.get_parameter('default_goal').value,
            fallback=np.zeros(4, dtype=np.float32),
        )
        self.cmd_vel_limits = np.array(
            [
                float(self.get_parameter('cmd_vel_x_limit').value),
                float(self.get_parameter('cmd_vel_y_limit').value),
                float(self.get_parameter('cmd_vel_yaw_limit').value),
            ],
            dtype=np.float32,
        )

        self.nav_output_scale = float(self.get_parameter('nav_output_scale').value)

        self.base_lin_vel = np.zeros(3, dtype=np.float32)
        self.proj_gravity = np.zeros(3, dtype=np.float32)
        self.goal = self.default_goal.copy()

        self.lowstate_received = False
        self.odom_received = False
        self.goal_received = False
        self.nav_enabled = not self.require_start_button
        self.last_start_button = False
        now = self.get_clock().now()
        self.last_lowstate_time = now
        self.last_odom_time = now
        self.last_goal_time = now
        self.last_enable_time = now
        self.last_odom_frame_id = None
        self.last_odom_child_frame_id = None
        self.ort_session = None

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, self.goal_topic, self.goal_callback, 10)
        self.create_subscription(Button, 'buttons', self.buttons_callback, 10)

        self.load_onnx_model(policy_name)
        self.timer = self.create_timer(1.0 / policy_frequency, self.publish_cmd_vel)

        self.get_logger().info(
            f'Navigation policy setup: name={policy_name} freq={policy_frequency}Hz'
        )
        self.get_logger().info(
            f'Odom gating: topic={self.odom_topic} timeout={self.odom_timeout_sec:.2f}s'
        )
        self.get_logger().info(
            f'Lowstate gating: timeout={self.lowstate_timeout_sec:.2f}s'
        )
        self.get_logger().info(
            f'Goal input: topic={self.goal_topic} timeout={self.goal_timeout_sec:.2f}s '
            f'default={self.default_goal.tolist()}'
        )
        if self.require_start_button:
            self.get_logger().info(
                'Navigation activation: press START after odom calibration to enable nav commands'
            )
        else:
            self.get_logger().info(
                'Navigation activation: fresh nav goals are accepted immediately'
            )
        self.get_logger().info(
            'Navigation cmd_vel limits: '
            f'x={self.cmd_vel_limits[0]:.2f} '
            f'y={self.cmd_vel_limits[1]:.2f} '
            f'yaw={self.cmd_vel_limits[2]:.2f}'
        )
        self.get_logger().info(
            f'Navigation output scale: {self.nav_output_scale:.3f}'
        )

    def _coerce_goal_vector(self, values, fallback):
        try:
            goal = np.asarray(values, dtype=np.float32).flatten()
        except (TypeError, ValueError):
            return fallback.copy()

        if goal.size < 4:
            return fallback.copy()

        goal = goal[:4]
        if not np.all(np.isfinite(goal)):
            return fallback.copy()

        return goal

    def lowstate_callback(self, msg):
        self.lowstate_received = True
        self.last_lowstate_time = self.get_clock().now()
        quat_wxyz = np.array(msg.imu_state.quaternion[0:4], dtype=np.float32)
        self.proj_gravity = self.body_projected_gravity_inverse(quat_wxyz)

    def odom_callback(self, msg):
        self.base_lin_vel = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            dtype=np.float32,
        )
        self.last_odom_time = self.get_clock().now()
        if not self.odom_received:
            self.get_logger().info(f'Received first odom on {self.odom_topic}')
        if (
            msg.header.frame_id != self.last_odom_frame_id
            or msg.child_frame_id != self.last_odom_child_frame_id
        ):
            self.get_logger().info(
                f'Odom frames updated: frame_id="{msg.header.frame_id}" '
                f'child_frame_id="{msg.child_frame_id}"'
            )
            self.last_odom_frame_id = msg.header.frame_id
            self.last_odom_child_frame_id = msg.child_frame_id
        self.odom_received = True

    def goal_callback(self, msg):
        try:
            raw_goal = np.asarray(msg.data, dtype=np.float32).flatten()
        except (TypeError, ValueError):
            raw_goal = np.array([], dtype=np.float32)

        if raw_goal.size < 4:
            self.get_logger().warn(
                f'Ignoring nav goal on {self.goal_topic}: expected 4 values, got {len(msg.data)}',
                throttle_duration_sec=2.0,
            )
            return

        goal = raw_goal[:4]
        if not np.all(np.isfinite(goal)):
            self.get_logger().warn(
                f'Ignoring nav goal on {self.goal_topic}: values must be finite',
                throttle_duration_sec=2.0,
            )
            return

        self.goal = goal
        self.goal_received = True
        self.last_goal_time = self.get_clock().now()

    def buttons_callback(self, msg):
        start_pressed = bool(msg.start)
        if start_pressed and not self.last_start_button:
            now = self.get_clock().now()
            self.nav_enabled = True
            self.last_enable_time = now
            self.get_logger().info(
                'Navigation enabled by START button; default launch goal window reset'
            )
        self.last_start_button = start_pressed

    def _elapsed_sec(self, now, since_time):
        return (now - since_time).nanoseconds * 1e-9

    def _fresh_goal_available(self, now):
        if not self.goal_received:
            return False
        goal_age = self._elapsed_sec(now, self.last_goal_time)
        return goal_age <= self.goal_timeout_sec

    def _launch_goal_active(self, now):
        if not self.nav_enabled:
            return False
        if not np.any(np.abs(self.default_goal) > 0.0):
            return False
        enabled_age = self._elapsed_sec(now, self.last_enable_time)
        return enabled_age <= self.goal_timeout_sec

    def _goal_vector(self, now):
        if self._fresh_goal_available(now):
            return self.goal

        if self.goal_received:
            goal_age = self._elapsed_sec(now, self.last_goal_time)
            self.get_logger().warn(
                f'Nav goal stale ({goal_age:.3f}s > {self.goal_timeout_sec:.3f}s).',
                throttle_duration_sec=2.0,
            )

        if self._launch_goal_active(now):
            return self.default_goal

        if np.any(np.abs(self.default_goal) > 0.0):
            enabled_age = self._elapsed_sec(now, self.last_enable_time)
            self.get_logger().warn(
                f'Launch nav goal expired ({enabled_age:.3f}s > {self.goal_timeout_sec:.3f}s). '
                'Publishing zero nav cmd_vel until a fresh /nav_goal arrives or START is pressed again.',
                throttle_duration_sec=2.0,
            )
        else:
            self.get_logger().warn(
                f'Waiting for {self.goal_topic}; no active nav goal yet.',
                throttle_duration_sec=2.0,
            )
        return None

    def _nav_ready(self):
        if self.nav_enabled:
            return True

        if not getattr(self, 'require_start_button', True):
            self.nav_enabled = True
            return True

        self.get_logger().info(
            'Navigation idle until START is pressed after odom calibration',
            throttle_duration_sec=2.0,
        )
        return False

    def _obs_ready(self, now):
        if not self._nav_ready():
            return False

        if not self.lowstate_received:
            self.get_logger().warn(
                'Waiting for /lowstate before nav policy inference',
                throttle_duration_sec=2.0,
            )
            return False

        lowstate_age = self._elapsed_sec(now, self.last_lowstate_time)
        if lowstate_age > self.lowstate_timeout_sec:
            self.get_logger().warn(
                f'/lowstate stale ({lowstate_age:.3f}s > {self.lowstate_timeout_sec:.3f}s). '
                'Publishing zero nav cmd_vel.',
                throttle_duration_sec=2.0,
            )
            return False

        if not self.odom_received:
            self.get_logger().warn(
                f'Waiting for {self.odom_topic} before nav policy inference',
                throttle_duration_sec=2.0,
            )
            return False

        odom_age = self._elapsed_sec(now, self.last_odom_time)
        if odom_age > self.odom_timeout_sec:
            self.get_logger().warn(
                f'Odom stale ({odom_age:.3f}s > {self.odom_timeout_sec:.3f}s). '
                'Publishing zero nav cmd_vel.',
                throttle_duration_sec=2.0,
            )
            return False

        return True

    def _zero_twist(self):
        return Twist()

    def _publish_twist(self, values):
        twist = Twist()
        twist.linear.x = float(values[0])
        twist.linear.y = float(values[1])
        twist.angular.z = float(values[2])
        self.publisher.publish(twist)

    def publish_cmd_vel(self):
        if self.ort_session is None:
            self.publisher.publish(self._zero_twist())
            return

        now = self.get_clock().now()
        if not self._obs_ready(now):
            self.publisher.publish(self._zero_twist())
            return

        goal = self._goal_vector(now)
        if goal is None:
            self.publisher.publish(self._zero_twist())
            return

        obs = np.zeros(10, dtype=np.float32)
        obs[0:3] = self.base_lin_vel
        obs[3:6] = self.proj_gravity
        obs[6:10] = goal
        input_obs = obs.reshape(1, -1)

        try:
            ort_inputs = {self.ort_session.get_inputs()[0].name: input_obs}
            output = self.ort_session.run(None, ort_inputs)[0].flatten().astype(np.float32)
        except Exception as exc:
            self.get_logger().error(f'Navigation inference failed: {exc}')
            self.publisher.publish(self._zero_twist())
            return

        if output.size != 3 or not np.all(np.isfinite(output)):
            self.get_logger().error(
                f'Navigation policy returned invalid cmd_vel vector: {output.tolist()}'
            )
            self.publisher.publish(self._zero_twist())
            return

        # apply scaling to policy outputs before clipping
        try:
            scaled_output = (output * self.nav_output_scale).astype(np.float32)
        except Exception as exc:
            self.get_logger().error(f'Navigation output scaling failed: {exc}')
            self.publisher.publish(self._zero_twist())
            return

        if not np.all(np.isfinite(scaled_output)):
            self.get_logger().error(
                f'Navigation scaled cmd_vel contains non-finite values: {scaled_output.tolist()}'
            )
            self.publisher.publish(self._zero_twist())
            return

        clipped = np.clip(scaled_output, -self.cmd_vel_limits, self.cmd_vel_limits)
        self._publish_twist(clipped)

    def body_projected_gravity_inverse(self, quat_wxyz):
        g_norm_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        rot = Rot.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        return (rot.inv().as_matrix() @ g_norm_world).astype(np.float32)

    def _extract_last_dim(self, shape):
        if not shape:
            return None
        dim = shape[-1]
        try:
            return int(dim)
        except (TypeError, ValueError):
            return None

    def load_onnx_model(self, policy_name):
        share_dir = get_package_share_directory('blind_locomotion')
        model_path = os.path.join(share_dir, 'models', f'{policy_name}.onnx')
        self.get_logger().info(f'Loading navigation model: {model_path}')
        try:
            self.ort_session = ort.InferenceSession(model_path)
        except Exception as exc:
            self.get_logger().fatal(f'Failed to load nav ONNX model: {exc}')
            self.ort_session = None
            return

        input_shape = self.ort_session.get_inputs()[0].shape
        output_shape = self.ort_session.get_outputs()[0].shape
        input_dim = self._extract_last_dim(input_shape)
        output_dim = self._extract_last_dim(output_shape)

        if input_dim != 10:
            self.get_logger().fatal(
                f'Invalid nav model input dim: expected 10, got {input_shape}. Refusing to run.'
            )
            self.ort_session = None
            return

        if output_dim != 3:
            self.get_logger().fatal(
                f'Invalid nav model output dim: expected 3, got {output_shape}. '
                'Refusing to run.'
            )
            self.ort_session = None
            return

        self.get_logger().info(
            f'Successfully loaded navigation ONNX model: {policy_name} '
            f'(input={input_shape}, output={output_shape})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RLNavCommandsNode()
    spin_until_shutdown(node)


if __name__ == '__main__':
    main()
