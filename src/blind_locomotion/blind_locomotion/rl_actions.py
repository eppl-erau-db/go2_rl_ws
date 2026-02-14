#!/usr/bin/env python3
import os

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Float32, Float32MultiArray
from unitree_go.msg import LowState


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
        self.declare_parameter('default_hip_q', 0.0)
        self.declare_parameter('default_thigh_q', 0.8)
        self.declare_parameter('default_calf_q', -1.5)

        # Observation configuration.
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('base_height_timeout_sec', 0.5)
        self.declare_parameter('cmd_vel_deadband', 0.1)
        self.declare_parameter('debug_enabled', True)
        self.declare_parameter('debug_rate_hz', 5.0)
        self.declare_parameter('debug_publish_obs_topic', False)
        self.declare_parameter('debug_publish_action_topic', False)

        policy_name = str(self.get_parameter('policy_name').value)
        policy_frequency = int(self.get_parameter('policy_frequency').value)
        self.scale_factor = float(self.get_parameter('scale_factor').value)
        default_hip_q = float(self.get_parameter('default_hip_q').value)
        default_thigh_q = float(self.get_parameter('default_thigh_q').value)
        default_calf_q = float(self.get_parameter('default_calf_q').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.base_height_timeout_sec = float(self.get_parameter('base_height_timeout_sec').value)
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

        self.q_defaults = np.concatenate(
            (
                np.ones(4, dtype=np.float32) * default_hip_q,
                np.ones(4, dtype=np.float32) * default_thigh_q,
                np.ones(4, dtype=np.float32) * default_calf_q,
            ),
            axis=0,
        )

        # Dynamic state.
        self.base_lin_vel = np.zeros(3, dtype=np.float32)
        self.ang_speed = np.zeros(3, dtype=np.float32)
        self.proj_gravity = np.zeros(3, dtype=np.float32)
        self.velocity_commands = np.zeros(3, dtype=np.float32)
        self.q = np.zeros(12, dtype=np.float32)
        self.dq = np.zeros(12, dtype=np.float32)
        self.raw_action = np.zeros(12, dtype=np.float32)
        self.last_actions = np.zeros(12, dtype=np.float32)

        self.base_height = 0.0
        self.base_vel_z_from_height = 0.0

        self.lowstate_received = False
        self.odom_received = False
        self.base_height_received = False
        self.cmd_vel_received = False
        now = self.get_clock().now()
        self.last_lowstate_time = now
        self.last_odom_time = now
        self.last_base_height_time = now
        self.last_cmd_vel_time = now
        self.last_debug_time = now
        self.last_nonfinite_log_time = {}
        self.latest_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.last_odom_frame_id = None
        self.last_odom_child_frame_id = None

        self.publisher = self.create_publisher(Float32MultiArray, 'actions', 10)
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
        self.create_subscription(Float32, '/base_height', self.base_height_callback, 10)

        self.load_onnx_model(policy_name)

        self.timer = self.create_timer(1.0 / policy_frequency, self.generate_actions)

        self.get_logger().info(
            f'Policy setup: name={policy_name} freq={policy_frequency}Hz '
            f'scale={self.scale_factor}'
        )
        self.get_logger().info(
            f'Odom gating: topic={self.odom_topic} timeout={self.odom_timeout_sec:.2f}s'
        )
        self.get_logger().info(
            f'Base height gating: topic=/base_height timeout={self.base_height_timeout_sec:.2f}s'
        )
        self.get_logger().info(
            'Obs layout (49): [0:3] base_lin_vel, [3:6] base_ang_vel, '
            '[6] base_height, [7:10] projected_gravity, [10:13] velocity_commands, '
            '[13:25] joint_pos, [25:37] joint_vel, [37:49] actions'
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
        self.proj_gravity = self.body_projected_gravity_inverse(self.latest_quat_wxyz)

        self.q = np.array(
            [
                msg.motor_state[3].q,   # FL_hip
                msg.motor_state[0].q,   # FR_hip
                msg.motor_state[9].q,   # RL_hip
                msg.motor_state[6].q,   # RR_hip
                msg.motor_state[4].q,   # FL_thigh
                msg.motor_state[1].q,   # FR_thigh
                msg.motor_state[10].q,  # RL_thigh
                msg.motor_state[7].q,   # RR_thigh
                msg.motor_state[5].q,   # FL_calf
                msg.motor_state[2].q,   # FR_calf
                msg.motor_state[11].q,  # RL_calf
                msg.motor_state[8].q,   # RR_calf
            ],
            dtype=np.float32,
        ) - self.q_defaults

        self.dq = np.array(
            [
                msg.motor_state[3].dq,   # FL_hip
                msg.motor_state[0].dq,   # FR_hip
                msg.motor_state[9].dq,   # RL_hip
                msg.motor_state[6].dq,   # RR_hip
                msg.motor_state[4].dq,   # FL_thigh
                msg.motor_state[1].dq,   # FR_thigh
                msg.motor_state[10].dq,  # RL_thigh
                msg.motor_state[7].dq,   # RR_thigh
                msg.motor_state[5].dq,   # FL_calf
                msg.motor_state[2].dq,   # FR_calf
                msg.motor_state[11].dq,  # RL_calf
                msg.motor_state[8].dq,   # RR_calf
            ],
            dtype=np.float32,
        )

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

    def base_height_callback(self, msg):
        new_height = float(msg.data)
        now = self.get_clock().now()
        if self.base_height_received:
            dt = (now - self.last_base_height_time).nanoseconds * 1e-9
            if dt > 1e-6:
                self.base_vel_z_from_height = (new_height - self.base_height) / dt
        else:
            self.get_logger().info('Received first base_height on /base_height')
        self.base_height = new_height
        self.last_base_height_time = now
        self.base_height_received = True

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

        if not self.base_height_received:
            self.get_logger().warn(
                'Waiting for /base_height before policy inference',
                throttle_duration_sec=2.0,
            )
            return False

        base_height_age = (now - self.last_base_height_time).nanoseconds * 1e-9
        if base_height_age > self.base_height_timeout_sec:
            self.get_logger().warn(
                f'Base height stale ({base_height_age:.3f}s > {self.base_height_timeout_sec:.3f}s). '
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

        obs = np.zeros(49, dtype=np.float32)
        obs[0:3] = self.base_lin_vel
        obs[2] = self.base_vel_z_from_height
        obs[3:6] = self.ang_speed
        obs[6] = self.base_height
        obs[7:10] = self.proj_gravity
        obs[10:13] = self.velocity_commands
        obs[13:25] = self.q
        obs[25:37] = self.dq
        obs[37:49] = self.last_actions
        obs_slices = {
            'base_lin_vel': obs[0:3],
            'base_ang_vel': obs[3:6],
            'base_height': obs[6:7],
            'projected_gravity': obs[7:10],
            'velocity_commands': obs[10:13],
            'joint_pos': obs[13:25],
            'joint_vel': obs[25:37],
            'actions': obs[37:49],
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

        processed = (self.raw_action * self.scale_factor + self.q_defaults).tolist()
        processed_np = np.asarray(processed, dtype=np.float32)
        processed_actions_ordered = [
            processed[1],   # FR_hip
            processed[5],   # FR_thigh
            processed[9],   # FR_calf
            processed[0],   # FL_hip
            processed[4],   # FL_thigh
            processed[8],   # FL_calf
            processed[3],   # RR_hip
            processed[7],   # RR_thigh
            processed[11],  # RR_calf
            processed[2],   # RL_hip
            processed[6],   # RL_thigh
            processed[10],  # RL_calf
        ]

        action_msg = Float32MultiArray()
        action_msg.data = processed_actions_ordered
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
            base_height_age_ms = 1000.0 * self._elapsed_sec(now, self.last_base_height_time)
            cmd_vel_age_ms = (
                1000.0 * self._elapsed_sec(now, self.last_cmd_vel_time)
                if self.cmd_vel_received
                else -1.0
            )

            raw_min, raw_max, raw_norm = self._slice_stats(self.raw_action)
            proc_min, proc_max, proc_norm = self._slice_stats(processed_np)
            raw_mean = float(np.mean(self.raw_action))
            proc_mean = float(np.mean(processed_np))
            raw_abs_gt_one = int(np.count_nonzero(np.abs(self.raw_action) > 1.0))

            blv_min, blv_max, blv_norm = self._slice_stats(obs_slices['base_lin_vel'])
            bav_min, bav_max, bav_norm = self._slice_stats(obs_slices['base_ang_vel'])
            bh_val = float(obs_slices['base_height'][0])
            grav_min, grav_max, grav_norm = self._slice_stats(obs_slices['projected_gravity'])
            cmd_min, cmd_max, cmd_norm = self._slice_stats(obs_slices['velocity_commands'])
            q_min, q_max, q_norm = self._slice_stats(obs_slices['joint_pos'])
            dq_min, dq_max, dq_norm = self._slice_stats(obs_slices['joint_vel'])
            act_hist_min, act_hist_max, act_hist_norm = self._slice_stats(obs_slices['actions'])

            cmd_age_text = f'{cmd_vel_age_ms:.1f}' if cmd_vel_age_ms >= 0.0 else 'n/a'
            self.get_logger().info(
                '[Debug] '
                f'freshness_ms=(lowstate={lowstate_age_ms:.1f}, odom={odom_age_ms:.1f}, '
                f'base_height={base_height_age_ms:.1f}, cmd_vel={cmd_age_text}) '
                f'obs_stats='
                f'blv[min={blv_min:.3f},max={blv_max:.3f},norm={blv_norm:.3f}] '
                f'bav[min={bav_min:.3f},max={bav_max:.3f},norm={bav_norm:.3f}] '
                f'bh[val={bh_val:.3f}] '
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
        self.get_logger().info(f'Model path: {model_path}')
        try:
            self.ort_session = ort.InferenceSession(model_path)
        except Exception as exc:
            self.get_logger().fatal(f'Failed to load ONNX model: {exc}')
            self.ort_session = None
            return

        input_shape = self.ort_session.get_inputs()[0].shape
        output_shape = self.ort_session.get_outputs()[0].shape
        input_dim = self._extract_last_dim(input_shape)
        output_dim = self._extract_last_dim(output_shape)

        if input_dim != 49:
            self.get_logger().fatal(
                f'Invalid model input dim: expected 49, got {input_shape}. '
                'Refusing to run.'
            )
            self.ort_session = None
            return

        if output_dim != 12:
            self.get_logger().fatal(
                f'Invalid model output dim: expected 12, got {output_shape}. '
                'Refusing to run.'
            )
            self.ort_session = None
            return

        self.get_logger().info(
            f'Successfully loaded ONNX model: {policy_name} '
            f'(input={input_shape}, output={output_shape})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RLActionsNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
