#!/home/srge/workspaces/go2_rl_ws/.venv/bin/python3
"""
RL Locomotion Policy Inference Node

Observation space (48-dim):
  [0:3]   base_lin_vel      - Linear velocity (zeros, no estimator)
  [3:6]   base_ang_vel      - Angular velocity from IMU gyroscope
  [6:9]  projected_gravity  - Gravity vector in body frame
  [9:12] velocity_commands  - Target Velocity: from wireless controller
  [12:24] joint_pos         - Joint positions (Isaac Lab order, offset by defaults)
  [24:36] joint_vel         - Joint velocities (Isaac Lab order)
  [36:48] actions           - Last action output

Action space (12-dim):
  Raw actions in Isaac Lab joint order
"""
import os

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState


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

        policy_name = str(self.get_parameter('policy_name').value)
        policy_frequency = int(self.get_parameter('policy_frequency').value)
        self.scale_factor = float(self.get_parameter('scale_factor').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.cmd_vel_deadband = float(self.get_parameter('cmd_vel_deadband').value)

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
        self.last_nonfinite_log_time = {}
        self.latest_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.last_odom_frame_id = None
        self.last_odom_child_frame_id = None

        self.publisher = self.create_publisher(Float32MultiArray, 'actions', 10)

        self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

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
            'Obs layout (48): [0:3] base_lin_vel, [3:6] base_ang_vel, '
            '[6:9] projected_gravity, [9:12] velocity_commands, '
            '[12:24] joint_pos, [24:36] joint_vel, [36:48] actions'
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

        processed = (self.raw_action * self.scale_factor + self.q_defaults).tolist()
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

        self.last_actions = self.raw_action.copy()

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

        if input_dim != 48:
            self.get_logger().fatal(
                f'Invalid model input dim: expected 48, got {input_shape}. '
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
