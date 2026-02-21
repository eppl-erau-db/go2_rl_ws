#!/usr/bin/env python3
"""
RL Locomotion Policy Inference Node

Observation space (48-dim):
  [0:3]   base_lin_vel      - Linear velocity from odometry
  [3:6]   base_ang_vel      - Angular velocity from IMU gyroscope
  [6:9]   projected_gravity - Gravity vector in body frame (inverse convention)
  [9:12]  velocity_commands - Target velocity from wireless controller
  [12:24] joint_pos         - Joint positions (Isaac Lab order, offset by defaults)
  [24:36] joint_vel         - Joint velocities (Isaac Lab order)
  [36:48] actions           - Last action output

Action space (12-dim):
  Raw actions in Isaac Lab joint order
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState

from blind_locomotion.msg import JointPositionCommand

from blind_locomotion.gravity_utils import projected_gravity_inverse
from blind_locomotion.joint_reorder import (
    actions_to_unitree_order,
    read_joint_positions_isaac,
    read_joint_velocities_isaac,
)
from blind_locomotion.onnx_loader import load_onnx_policy


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class RLActionsNode(Node):
    def __init__(self):
        super().__init__('rl_actions_publisher')

        # Core policy parameters.
        self.declare_parameter('policy_name', 'locomotion_policy')
        self.declare_parameter('policy_frequency', 25)
        self.declare_parameter('scale_factor', 0.25)
        # Observation configuration.
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('cmd_vel_deadband', 0.1)
        self.declare_parameter('debug_enabled', True)
        self.declare_parameter('debug_rate_hz', 5.0)
        self.declare_parameter('debug_publish_obs_topic', False)
        self.declare_parameter('debug_publish_action_topic', False)

        policy_name = str(self.get_parameter('policy_name').value)
        policy_frequency = int(self.get_parameter('policy_frequency').value)
        self.scale_factor = float(self.get_parameter('scale_factor').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.cmd_vel_deadband = float(self.get_parameter('cmd_vel_deadband').value)
        self.debug_enabled = as_bool(self.get_parameter('debug_enabled').value)
        self.debug_rate_hz = max(0.1, float(self.get_parameter('debug_rate_hz').value))
        self.debug_publish_obs_topic = as_bool(
            self.get_parameter('debug_publish_obs_topic').value
        )
        self.debug_publish_action_topic = as_bool(
            self.get_parameter('debug_publish_action_topic').value
        )
        self.debug_interval_sec = 1.0 / self.debug_rate_hz

        # Default joint positions in Isaac Lab order (used as observation offset
        # and as the action-space center point).
        self.q_defaults = np.array([
             0.1, -0.1,  0.1, -0.1,   # hips:   FL, FR, RL, RR
             0.8,  0.8,  1.0,  1.0,    # thighs: FL, FR, RL, RR
            -1.5, -1.5, -1.5, -1.5,    # calfs:  FL, FR, RL, RR
        ], dtype=np.float32)

        # Dynamic state.
        self.base_lin_vel = np.zeros(3, dtype=np.float32)
        self.ang_speed = np.zeros(3, dtype=np.float32)
        self.proj_gravity = np.zeros(3, dtype=np.float32)
        self.velocity_commands = np.zeros(3, dtype=np.float32)
        self.q = np.zeros(12, dtype=np.float32)
        self.dq = np.zeros(12, dtype=np.float32)
        self.raw_action = np.zeros(12, dtype=np.float32)
        self.last_actions = np.zeros(12, dtype=np.float32)

        self.lowstate_received = False
        self.odom_received = False
        self.cmd_vel_received = False
        now = self.get_clock().now()
        self.last_lowstate_time = now
        self.last_odom_time = now
        self.last_cmd_vel_time = now
        self.last_debug_time = now
        self.last_nonfinite_log_time = {}
        self.latest_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.last_odom_frame_id = None
        self.last_odom_child_frame_id = None

        self.publisher = self.create_publisher(JointPositionCommand, 'actions', 10)
        self.debug_obs_publisher = None
        self.debug_raw_action_publisher = None
        if self.debug_publish_obs_topic:
            self.debug_obs_publisher = self.create_publisher(
                Float32MultiArray, 'debug/rl_obs', 10
            )
        if self.debug_publish_action_topic:
            self.debug_raw_action_publisher = self.create_publisher(
                Float32MultiArray, 'debug/rl_raw_action', 10
            )

        self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.ort_session = load_onnx_policy(
            self.get_logger(), policy_name,
            expected_input_dim=48, expected_output_dim=12,
        )

        self.timer = self.create_timer(1.0 / policy_frequency, self.generate_actions)

        self.get_logger().info(
            f'Policy setup: name={policy_name} freq={policy_frequency}Hz '
            f'scale={self.scale_factor}'
        )
        self.get_logger().info(
            f'Odom gating: topic={self.odom_topic} timeout={self.odom_timeout_sec:.2f}s'
        )
        self.get_logger().info(
            'Obs layout (48): [0:3] base_lin_vel, [3:6] base_ang_vel, '
            '[6:9] projected_gravity, [9:12] velocity_commands, '
            '[12:24] joint_pos, [24:36] joint_vel, [36:48] actions'
        )
        self.get_logger().info(
            'Debug config: '
            f'enabled={self.debug_enabled} rate={self.debug_rate_hz:.1f}Hz '
            f'publish_obs={self.debug_publish_obs_topic} '
            f'publish_raw_action={self.debug_publish_action_topic}'
        )
        self.get_logger().info('Gravity projection: mode=inverse (hardcoded)')

    def lowstate_callback(self, msg):
        self.lowstate_received = True
        self.last_lowstate_time = self.get_clock().now()

        self.ang_speed = np.array(msg.imu_state.gyroscope[0:3], dtype=np.float32)
        self.latest_quat_wxyz = np.array(msg.imu_state.quaternion[0:4], dtype=np.float32)
        self.proj_gravity = projected_gravity_inverse(self.latest_quat_wxyz)

        self.q = read_joint_positions_isaac(msg.motor_state) - self.q_defaults
        self.dq = read_joint_velocities_isaac(msg.motor_state)

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

    def cmd_vel_callback(self, msg):
        self.cmd_vel_received = True
        self.last_cmd_vel_time = self.get_clock().now()
        commands = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z],
            dtype=np.float32,
        )
        if np.all(np.abs(commands) < self.cmd_vel_deadband):
            commands = np.zeros(3, dtype=np.float32)
        self.velocity_commands = commands

    def _elapsed_sec(self, now, since_time):
        return (now - since_time).nanoseconds * 1e-9

    def _should_debug_log(self, now):
        if not self.debug_enabled:
            return False
        if self._elapsed_sec(now, self.last_debug_time) < self.debug_interval_sec:
            return False
        self.last_debug_time = now
        return True

    def _log_nonfinite_slice(self, name, values, now):
        finite_mask = np.isfinite(values)
        if finite_mask.all():
            return

        last_time = self.last_nonfinite_log_time.get(name)
        if last_time is not None and self._elapsed_sec(now, last_time) < 1.0:
            return

        self.last_nonfinite_log_time[name] = now
        local_bad = np.where(~finite_mask)[0].tolist()
        self.get_logger().error(
            f'Observation slice "{name}" has non-finite values at local indices {local_bad}: '
            f'{np.array2string(values, precision=4)}'
        )

    def _slice_stats(self, values):
        return (
            float(np.min(values)),
            float(np.max(values)),
            float(np.linalg.norm(values)),
        )

    def _obs_ready(self, now):
        if not self.lowstate_received:
            self.get_logger().warn('Waiting for /lowstate before policy inference', throttle_duration_sec=2.0)
            return False

        if not self.odom_received:
            self.get_logger().warn(
                f'Waiting for {self.odom_topic} before policy inference',
                throttle_duration_sec=2.0,
            )
            return False

        odom_age = (now - self.last_odom_time).nanoseconds * 1e-9
        if odom_age > self.odom_timeout_sec:
            self.get_logger().warn(
                f'Odom stale ({odom_age:.3f}s > {self.odom_timeout_sec:.3f}s). '
                'Pausing action publish.',
                throttle_duration_sec=2.0,
            )
            return False

        return True

    def generate_actions(self):
        if self.ort_session is None:
            self.get_logger().warn('ONNX model not loaded, skipping inference', once=True)
            return

        now = self.get_clock().now()
        if not self._obs_ready(now):
            return

        obs = np.zeros(48, dtype=np.float32)
        obs[0:3]   = self.base_lin_vel
        obs[3:6]   = self.ang_speed
        obs[6:9]   = self.proj_gravity
        obs[9:12]  = self.velocity_commands
        obs[12:24] = self.q
        obs[24:36] = self.dq
        obs[36:48] = self.last_actions
        obs_slices = {
            'base_lin_vel': obs[0:3],
            'base_ang_vel': obs[3:6],
            'projected_gravity': obs[6:9],
            'velocity_commands': obs[9:12],
            'joint_pos': obs[12:24],
            'joint_vel': obs[24:36],
            'actions': obs[36:48],
        }
        for name, values in obs_slices.items():
            self._log_nonfinite_slice(name, values, now)

        input_obs = obs.reshape(1, -1)

        try:
            ort_inputs = {self.ort_session.get_inputs()[0].name: input_obs}
            ort_outs = self.ort_session.run(None, ort_inputs)
            self.raw_action = ort_outs[0].flatten().astype(np.float32)
        except Exception as exc:
            self.get_logger().error(f'Inference failed: {exc}')
            self.raw_action = np.zeros(12, dtype=np.float32)

        processed = self.raw_action * self.scale_factor + self.q_defaults
        processed_unitree = actions_to_unitree_order(processed)

        action_msg = JointPositionCommand()
        action_msg.positions = processed_unitree
        self.publisher.publish(action_msg)
        if self.debug_obs_publisher is not None:
            obs_msg = Float32MultiArray()
            obs_msg.data = obs.tolist()
            self.debug_obs_publisher.publish(obs_msg)
        if self.debug_raw_action_publisher is not None:
            raw_action_msg = Float32MultiArray()
            raw_action_msg.data = self.raw_action.tolist()
            self.debug_raw_action_publisher.publish(raw_action_msg)

        self.last_actions = self.raw_action.copy()

        if self._should_debug_log(now):
            lowstate_age_ms = 1000.0 * self._elapsed_sec(now, self.last_lowstate_time)
            odom_age_ms = 1000.0 * self._elapsed_sec(now, self.last_odom_time)
            cmd_vel_age_ms = (
                1000.0 * self._elapsed_sec(now, self.last_cmd_vel_time)
                if self.cmd_vel_received
                else -1.0
            )

            raw_min, raw_max, raw_norm = self._slice_stats(self.raw_action)
            proc_min, proc_max, proc_norm = self._slice_stats(processed)
            raw_mean = float(np.mean(self.raw_action))
            proc_mean = float(np.mean(processed))
            raw_abs_gt_one = int(np.count_nonzero(np.abs(self.raw_action) > 1.0))

            blv_min, blv_max, blv_norm = self._slice_stats(obs_slices['base_lin_vel'])
            bav_min, bav_max, bav_norm = self._slice_stats(obs_slices['base_ang_vel'])
            grav_min, grav_max, grav_norm = self._slice_stats(obs_slices['projected_gravity'])
            cmd_min, cmd_max, cmd_norm = self._slice_stats(obs_slices['velocity_commands'])
            q_min, q_max, q_norm = self._slice_stats(obs_slices['joint_pos'])
            dq_min, dq_max, dq_norm = self._slice_stats(obs_slices['joint_vel'])
            act_hist_min, act_hist_max, act_hist_norm = self._slice_stats(obs_slices['actions'])

            cmd_age_text = f'{cmd_vel_age_ms:.1f}' if cmd_vel_age_ms >= 0.0 else 'n/a'
            self.get_logger().info(
                '[Debug] '
                f'freshness_ms=(lowstate={lowstate_age_ms:.1f}, odom={odom_age_ms:.1f}, '
                f'cmd_vel={cmd_age_text}) '
                f'obs_stats='
                f'blv[min={blv_min:.3f},max={blv_max:.3f},norm={blv_norm:.3f}] '
                f'bav[min={bav_min:.3f},max={bav_max:.3f},norm={bav_norm:.3f}] '
                f'grav[min={grav_min:.3f},max={grav_max:.3f},norm={grav_norm:.3f}] '
                f'cmd[min={cmd_min:.3f},max={cmd_max:.3f},norm={cmd_norm:.3f}] '
                f'q[min={q_min:.3f},max={q_max:.3f},norm={q_norm:.3f}] '
                f'dq[min={dq_min:.3f},max={dq_max:.3f},norm={dq_norm:.3f}] '
                f'actions[min={act_hist_min:.3f},max={act_hist_max:.3f},norm={act_hist_norm:.3f}] '
                f'raw_action[min={raw_min:.3f},max={raw_max:.3f},mean={raw_mean:.3f},'
                f'norm={raw_norm:.3f},abs_gt_1={raw_abs_gt_one}] '
                f'processed[min={proc_min:.3f},max={proc_max:.3f},mean={proc_mean:.3f},'
                f'norm={proc_norm:.3f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RLActionsNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
