#!/home/srge/workspaces/go2_rl_ws/.venv/bin/python3
"""
RL Reaching/Standing Policy Inference Node

Observation space (53-dim):
  [0:3]   base_lin_vel      - Linear velocity from odometry (/odom by default)
  [3:6]   base_ang_vel      - Angular velocity from IMU gyroscope
  [6:7]   base_height       - Base height from /base_height (startup default 0.25m)
  [7:10]  projected_gravity - Gravity vector in body frame
  [10:17] pose_command      - Target pose: position(3) + quaternion(4)
  [17:29] joint_pos         - Joint positions (Isaac Lab order, offset by defaults)
  [29:41] joint_vel         - Joint velocities (Isaac Lab order)
  [41:53] actions           - Last action output

Action space (12-dim):
  Raw actions in Isaac Lab joint order, scaled and offset before publishing

Inference is gated on both /lowstate freshness and odometry freshness.
"""
import os
import rclpy
import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from unitree_go.msg import LowState
from ament_index_python.packages import get_package_share_directory

try:
    from blind_locomotion.ros_node_shutdown import spin_until_shutdown
except ModuleNotFoundError:
    from ros_node_shutdown import spin_until_shutdown


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def ordered_range(lo, hi):
    if lo <= hi:
        return (lo, hi)
    return (hi, lo)


class RLReachActionsNode(Node):
    def __init__(self):
        super().__init__("rl_reach_actions_publisher")

        # params
        self.declare_parameter("policy_name", "SimplePolicy")
        self.declare_parameter("policy_frequency", 50)
        self.declare_parameter("scale_factor", 0.25)
        self.declare_parameter("default_hip_q", 0.0)
        self.declare_parameter("default_thigh_q", 0.8)
        self.declare_parameter("default_calf_q", -1.5)
        self.declare_parameter("base_height_topic", "/base_height")
        self.declare_parameter("lowstate_timeout_sec", 0.5)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("pose_command_topic", "pose_command")
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("pose_x_min", 0.20)
        self.declare_parameter("pose_x_max", 0.50)
        self.declare_parameter("pose_y_min", -0.30)
        self.declare_parameter("pose_y_max", -0.15)
        self.declare_parameter("pose_z_min", 0.0)
        self.declare_parameter("pose_z_max", 0.30)

        policy_name = self.get_parameter("policy_name").get_parameter_value().string_value
        policy_frequency = self.get_parameter("policy_frequency").get_parameter_value().integer_value
        scale_factor = self.get_parameter("scale_factor").get_parameter_value().double_value
        default_hip_q = self.get_parameter("default_hip_q").get_parameter_value().double_value
        default_thigh_q = self.get_parameter("default_thigh_q").get_parameter_value().double_value
        default_calf_q = self.get_parameter("default_calf_q").get_parameter_value().double_value
        self.base_height_topic = self.get_parameter("base_height_topic").get_parameter_value().string_value
        self.lowstate_timeout_sec = self.get_parameter("lowstate_timeout_sec").get_parameter_value().double_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.odom_timeout_sec = self.get_parameter("odom_timeout_sec").get_parameter_value().double_value
        self.pose_command_topic = self.get_parameter("pose_command_topic").get_parameter_value().string_value
        self.pose_timeout_sec = self.get_parameter("pose_timeout_sec").get_parameter_value().double_value
        self.pose_x_range = ordered_range(
            self.get_parameter("pose_x_min").get_parameter_value().double_value,
            self.get_parameter("pose_x_max").get_parameter_value().double_value,
        )
        self.pose_y_range = ordered_range(
            self.get_parameter("pose_y_min").get_parameter_value().double_value,
            self.get_parameter("pose_y_max").get_parameter_value().double_value,
        )
        self.pose_z_range = ordered_range(
            self.get_parameter("pose_z_min").get_parameter_value().double_value,
            self.get_parameter("pose_z_max").get_parameter_value().double_value,
        )

        # publisher
        self.publisher = self.create_publisher(Float32MultiArray, 'actions', 10)

        # subscribers
        self.subscription = self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.pose_subscription = self.create_subscription(
            Pose,
            self.pose_command_topic,
            self.pose_command_callback,
            10,
        )
        self.base_height_subscription = self.create_subscription(
            Float32,
            self.base_height_topic,
            self.base_height_callback,
            10,
        )

        # load policy and set inference frequency
        self.load_onnx_model(policy_name)
        self.timer = self.create_timer(1 / policy_frequency, self.generate_actions)

        # Isaac Lab constants
        self.scale_factor = scale_factor
        self.q_defaults = np.concatenate((
            np.ones(4) * default_hip_q,
            np.ones(4) * default_thigh_q,
            np.ones(4) * default_calf_q
        ), axis=0).astype(np.float32)

        # Fixed observation components
        self.base_lin_vel = np.zeros(3, dtype=np.float32)
        self.base_height = np.float32(0.25)
        self.neutral_pose_command = self._neutral_pose_command()
        self.pose_command = self.neutral_pose_command.copy()
        self.pose_command_received = False
        now = self.get_clock().now()
        self.last_pose_command_time = now

        # Dynamic observation components (updated from lowstate)
        self.ang_speed = np.zeros(3, dtype=np.float32)
        self.proj_gravity = np.zeros(3, dtype=np.float32)
        self.q = np.zeros(12, dtype=np.float32)
        self.dq = np.zeros(12, dtype=np.float32)
        self.raw_action = np.zeros(12, dtype=np.float32)
        self.lowstate_received = False
        self.odom_received = False
        self.last_lowstate_time = now
        self.last_odom_time = now
        self.last_odom_frame_id = None
        self.last_odom_child_frame_id = None

        self.get_logger().info(f"Reach policy node started with {policy_name}")
        self.get_logger().info(
            f"Pose command: topic={self.pose_command_topic} timeout={self.pose_timeout_sec:.2f}s "
            f"x=({self.pose_x_range[0]:.3f}, {self.pose_x_range[1]:.3f}) "
            f"y=({self.pose_y_range[0]:.3f}, {self.pose_y_range[1]:.3f}) "
            f"z=({self.pose_z_range[0]:.3f}, {self.pose_z_range[1]:.3f})"
        )
        self.get_logger().info(
            f"Base-height input: topic={self.base_height_topic} startup_default=0.25"
        )
        self.get_logger().info(
            f"Lowstate gating: timeout={self.lowstate_timeout_sec:.2f}s"
        )
        self.get_logger().info(
            f"Odom gating: topic={self.odom_topic} timeout={self.odom_timeout_sec:.2f}s"
        )

    def lowstate_callback(self, msg):
        self.lowstate_received = True
        self.last_lowstate_time = self.get_clock().now()

        # base angular velocity from IMU
        self.ang_speed = np.array(msg.imu_state.gyroscope[0:3], dtype=np.float32)

        # projected gravity vector from IMU quaternion
        self.proj_gravity = self.body_projected_gravity(
            np.array(msg.imu_state.quaternion[0:4], dtype=np.float32)
        )

        # joint positions in Isaac Lab order (with defaults subtracted)
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

        # joint velocities in Isaac Lab order
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
            self.get_logger().info(f"Received first odom on {self.odom_topic}")
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
        self.base_height = np.float32(msg.data)

    def _neutral_pose_command(self):
        pose_x = self.pose_x_range[0]
        pose_y = self.pose_y_range[1]
        pose_z = self.pose_z_range[0]
        return np.array([pose_x, pose_y, pose_z, 1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    def pose_command_callback(self, msg):
        self.pose_command_received = True
        self.last_pose_command_time = self.get_clock().now()

        pose_x = clamp(float(msg.position.x), self.pose_x_range[0], self.pose_x_range[1])
        pose_y = clamp(float(msg.position.y), self.pose_y_range[0], self.pose_y_range[1])
        pose_z = clamp(float(msg.position.z), self.pose_z_range[0], self.pose_z_range[1])
        quat_wxyz = np.array(
            [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(quat_wxyz)):
            quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        self.pose_command = np.array(
            [pose_x, pose_y, pose_z, quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]],
            dtype=np.float32,
        )

    def _refresh_pose_command(self):
        if not self.pose_command_received:
            self.pose_command = self.neutral_pose_command.copy()
            return

        pose_age = (self.get_clock().now() - self.last_pose_command_time).nanoseconds * 1e-9
        if pose_age > self.pose_timeout_sec:
            self.pose_command = self.neutral_pose_command.copy()

    def _elapsed_sec(self, now, since_time):
        return (now - since_time).nanoseconds * 1e-9

    def _obs_ready(self, now):
        if not self.lowstate_received:
            self.get_logger().warn(
                "Waiting for /lowstate before policy inference",
                throttle_duration_sec=2.0,
            )
            return False

        lowstate_age = self._elapsed_sec(now, self.last_lowstate_time)
        if lowstate_age > self.lowstate_timeout_sec:
            self.get_logger().warn(
                f"/lowstate stale ({lowstate_age:.3f}s > {self.lowstate_timeout_sec:.3f}s). "
                "Pausing action publish.",
                throttle_duration_sec=2.0,
            )
            return False

        if not self.odom_received:
            self.get_logger().warn(
                f"Waiting for {self.odom_topic} before policy inference",
                throttle_duration_sec=2.0,
            )
            return False

        odom_age = self._elapsed_sec(now, self.last_odom_time)
        if odom_age > self.odom_timeout_sec:
            self.get_logger().warn(
                f"Odom stale ({odom_age:.3f}s > {self.odom_timeout_sec:.3f}s). "
                "Pausing action publish.",
                throttle_duration_sec=2.0,
            )
            return False

        return True

    def generate_actions(self):
        if self.ort_session is None:
            self.get_logger().warn("ONNX model not loaded, skipping inference")
            return

        now = self.get_clock().now()
        if not self._obs_ready(now):
            return

        self._refresh_pose_command()

        # Build 53-dim observation
        obs = np.zeros(53, dtype=np.float32)
        obs[0:3] = self.base_lin_vel       # base_lin_vel (3)
        obs[3:6] = self.ang_speed          # base_ang_vel (3)
        obs[6] = self.base_height          # base_height (1)
        obs[7:10]  = self.proj_gravity      # projected_gravity (3)
        obs[10:17] = self.pose_command      # pose_command (7)
        obs[17:29] = self.q                 # joint_pos (12)
        obs[29:41] = self.dq                # joint_vel (12)
        obs[41:53] = self.raw_action        # actions (12)

        input_obs = obs.reshape(1, -1)

        # Run ONNX inference
        try:
            ort_inputs = {self.ort_session.get_inputs()[0].name: input_obs}
            ort_outs = self.ort_session.run(None, ort_inputs)
            self.raw_action = ort_outs[0].flatten().astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.raw_action = np.zeros(12, dtype=np.float32)

        # Apply scale and offset, then reorder to Unitree joint order
        processed = (self.raw_action * self.scale_factor + self.q_defaults).tolist()
        unitree_order = [
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

        # Publish action
        action_msg = Float32MultiArray()
        action_msg.data = unitree_order
        self.publisher.publish(action_msg)

    def body_projected_gravity(self, quat_wxyz):
        """Compute gravity vector in body frame from IMU quaternion."""
        g_norm_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        # IMU gives [w,x,y,z], SciPy expects [x,y,z,w].
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
        self.get_logger().info(f"Loading model: {model_path}")
        try:
            self.ort_session = ort.InferenceSession(model_path)
        except Exception as e:
            self.get_logger().fatal(f"Failed to load ONNX model: {e}")
            self.ort_session = None
            return

        input_shape = self.ort_session.get_inputs()[0].shape
        output_shape = self.ort_session.get_outputs()[0].shape
        input_dim = self._extract_last_dim(input_shape)
        output_dim = self._extract_last_dim(output_shape)

        if input_dim != 53:
            self.get_logger().fatal(
                f"Invalid model input dim: expected 53, got {input_shape}. Refusing to run."
            )
            self.ort_session = None
            return

        if output_dim != 12:
            self.get_logger().fatal(
                f"Invalid model output dim: expected 12, got {output_shape}. Refusing to run."
            )
            self.ort_session = None
            return

        self.get_logger().info(
            f"Successfully loaded ONNX model: {policy_name} "
            f"(input={input_shape}, output={output_shape})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RLReachActionsNode()
    spin_until_shutdown(node)


if __name__ == "__main__":
    main()
