#!/home/srge/workspaces/go2_rl_ws/.venv/bin/python3
"""
RL Reaching/Standing Policy Inference Node

Observation space (53-dim):
  [0:3]   base_lin_vel      - Linear velocity (zeros, no estimator)
  [3:6]   base_ang_vel      - Angular velocity from IMU gyroscope
  [6:7]   base_height       - Base height (constant 0.25m)
  [7:10]  projected_gravity - Gravity vector in body frame
  [10:17] pose_command      - Target pose: position(3) + quaternion(4)
  [17:29] joint_pos         - Joint positions (Isaac Lab order, offset by defaults)
  [29:41] joint_vel         - Joint velocities (Isaac Lab order)
  [41:53] actions           - Last action output

Action space (12-dim):
  Raw actions in Isaac Lab joint order, scaled and offset before publishing
"""
import os
import rclpy
import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation as Rot 
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState
from ament_index_python.packages import get_package_share_directory


class RLReachActionsNode(Node):
    def __init__(self):
        super().__init__("rl_reach_actions_publisher")

        # params
        self.declare_parameter("policy_name", "SimplePolicy")
        self.declare_parameter("policy_frequency", 25)
        self.declare_parameter("scale_factor", 0.25)
        self.declare_parameter("default_hip_q", 0.0)
        self.declare_parameter("default_thigh_q", 0.8)
        self.declare_parameter("default_calf_q", -1.5)
        self.declare_parameter("base_height", 0.25)
        
        policy_name = self.get_parameter("policy_name").get_parameter_value().string_value
        policy_frequency = self.get_parameter("policy_frequency").get_parameter_value().integer_value
        scale_factor = self.get_parameter("scale_factor").get_parameter_value().double_value
        default_hip_q = self.get_parameter("default_hip_q").get_parameter_value().double_value
        default_thigh_q = self.get_parameter("default_thigh_q").get_parameter_value().double_value
        default_calf_q = self.get_parameter("default_calf_q").get_parameter_value().double_value
        self.base_height = self.get_parameter("base_height").get_parameter_value().double_value

        # publisher
        self.publisher = self.create_publisher(Float32MultiArray, 'actions', 10)
        
        # subscriber
        self.subscription = self.create_subscription(
            LowState, '/lowstate', self.lowstate_callback, 10)

        # load policy and set inference frequency
        self.load_onnx_model(policy_name)
        self.timer = self.create_timer(1/policy_frequency, self.generate_actions)
        
        # Isaac Lab constants
        self.scale_factor = scale_factor
        self.q_defaults = np.concatenate((
            np.ones(4) * default_hip_q,
            np.ones(4) * default_thigh_q,
            np.ones(4) * default_calf_q
        ), axis=0)

        # Fixed observation components
        self.base_lin_vel = np.zeros(3)  # No velocity estimator yet
        self.pose_command = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])  # Identity pose

        # Dynamic observation components (updated from lowstate)
        self.ang_speed = np.zeros(3)
        self.proj_gravity = np.zeros(3)
        self.q = np.zeros(12)
        self.dq = np.zeros(12)
        self.raw_action = np.zeros(12)
        
        self.get_logger().info(f"Reach policy node started with {policy_name}")
        
    def lowstate_callback(self, msg):
        # base angular velocity from IMU
        self.ang_speed = np.array(msg.imu_state.gyroscope[0:3])

        # projected gravity vector from IMU quaternion
        self.proj_gravity = self.body_projected_gravity(
            np.array(msg.imu_state.quaternion[0:4]))
        
        # joint positions in Isaac Lab order (with defaults subtracted)
        self.q = np.array([
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
        ]) - self.q_defaults

        # joint velocities in Isaac Lab order
        self.dq = np.array([
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
        ])

    def generate_actions(self):
        if self.ort_session is None:
            self.get_logger().warn("ONNX model not loaded, skipping inference")
            return
            
        # Build 53-dim observation
        obs = np.zeros(53, dtype=np.float32)
        obs[0:3] = self.base_lin_vel       # base_lin_vel (3)
        obs[3:6] = self.ang_speed          # base_ang_vel (3)
        obs[6]     = self.base_height        # base_height (1)
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
            self.raw_action = ort_outs[0].flatten()
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.raw_action = np.zeros(12)

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

    def body_projected_gravity(self, quat):
        """Compute gravity vector in body frame from IMU quaternion.
        
        Args:
            quat: Quaternion from IMU in [w, x, y, z] format
        Returns:
            Gravity vector in body frame (3,)
        """
        g_norm_world = np.array([0.0, 0.0, -1.0])
        # IMU gives [w,x,y,z], SciPy expects [x,y,z,w]
        rot = Rot.from_quat([quat[1], quat[2], quat[3], quat[0]])
        return rot.as_matrix() @ g_norm_world

    def load_onnx_model(self, policy_name):
        share_dir = get_package_share_directory('blind_locomotion')
        model_path = os.path.join(share_dir, 'models', f'{policy_name}.onnx')
        self.get_logger().info(f"Loading model: {model_path}")
        try:
            self.ort_session = ort.InferenceSession(model_path)
            self.get_logger().info(f"Successfully loaded ONNX model: {policy_name}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load ONNX model: {e}")
            self.ort_session = None   


def main(args=None):
    rclpy.init(args=args)
    node = RLReachActionsNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
