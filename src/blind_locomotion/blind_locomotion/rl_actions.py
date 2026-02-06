#!/usr/bin/env python3
import os
import rclpy
import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation as Rot 
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from unitree_go.msg import LowState
from ament_index_python.packages import get_package_share_directory


class RLActionsNode(Node):
    def __init__(self):
        super().__init__("rl_actions_publisher")

        # params
        self.declare_parameter("policy_name")
        self.declare_parameter("policy_frequency", 25)
        self.declare_parameter("scale_factor", 0.25)
        self.declare_parameter("default_hip_q", 0.0)
        self.declare_parameter("default_thigh_q", 1.1)
        self.declare_parameter("default_calf_q", -1.8)
        policy_name = self.get_parameter("policy_name").get_parameter_value().string_value
        policy_frequency = self.get_parameter("policy_frequency").get_parameter_value().integer_value
        scale_factor = self.get_parameter("scale_factor").get_parameter_value().double_value
        default_hip_q = self.get_parameter("default_hip_q").get_parameter_value().double_value
        default_thigh_q = self.get_parameter("default_thigh_q").get_parameter_value().double_value
        default_calf_q = self.get_parameter("default_calf_q").get_parameter_value().double_value

        # publisher
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'actions',
            10)
        
        # subscribers
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate', 
            self.lowstate_callback,
            10)
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # loading policy and settine frequency
        self.load_onnx_model(policy_name)
        self.timer = self.create_timer(
            1/policy_frequency,
            self.generate_actions
        )
        
        # isaac lab constants
        self.scale_factor = scale_factor
        self.q_defaults = np.concatenate(
            (np.ones(4)*default_hip_q,
             np.ones(4)*default_thigh_q,
             np.ones(4)*default_calf_q),
            axis=0
        )

        # init obs
        self.ang_speed = np.zeros(3)
        self.proj_gravity = np.zeros(3)
        self.cmd_vel = np.zeros(3)
        self.q = np.zeros(12)
        self.dq = np.zeros(12)
        self.raw_action = np.zeros(12)
        
    def lowstate_callback(self, msg):
        # base angular velocity
        self.ang_speed = np.array(msg.imu_state.gyroscope[0:3])

        # projected gravity vector
        self.proj_gravity = self.body_projected_gravity(np.array(msg.imu_state.quaternion[0:4]))
        
        # joint positions
        self.q = np.array([
            msg.motor_state[3].q,    # 0  -> FL_hip_joint   to FL_hip   -> 3
            msg.motor_state[0].q,    # 1  -> FR_hip_joint   to FR_hip   -> 0
            msg.motor_state[9].q,    # 2  -> RL_hip_joint   to RL_hip   -> 9
            msg.motor_state[6].q,    # 3  -> RR_hip_joint   to RR_hip   -> 6
            msg.motor_state[4].q,    # 4  -> FL_thigh_joint to FL_thigh -> 4
            msg.motor_state[1].q,    # 5  -> FR_thigh_joint to FR_thigh -> 1
            msg.motor_state[10].q,   # 6  -> RL_thigh_joint to RL_thigh -> 10
            msg.motor_state[7].q,    # 7  -> RR_thigh_joint to RR_thigh -> 7
            msg.motor_state[5].q,    # 8  -> FL_calf_joint  to FL_calf  -> 5
            msg.motor_state[2].q,    # 9  -> FR_calf_joint  to FR_calf  -> 2
            msg.motor_state[11].q,   # 10 -> RL_calf_joint  to RL_calf  -> 11
            msg.motor_state[8].q,    # 11 -> RR_calf_joint  to RR_calf  -> 8
        ]) - self.q_defaults

        # joint velocities
        self.dq = np.array([
            msg.motor_state[3].dq,   # 0  -> FL_hip_joint   to FL_hip   -> 3
            msg.motor_state[0].dq,   # 1  -> FR_hip_joint   to FR_hip   -> 0
            msg.motor_state[9].dq,   # 2  -> RL_hip_joint   to RL_hip   -> 9
            msg.motor_state[6].dq,   # 3  -> RR_hip_joint   to RR_hip   -> 6
            msg.motor_state[4].dq,   # 4  -> FL_thigh_joint to FL_thigh -> 4
            msg.motor_state[1].dq,   # 5  -> FR_thigh_joint to FR_thigh -> 1
            msg.motor_state[10].dq,  # 6  -> RL_thigh_joint to RL_thigh -> 10
            msg.motor_state[7].dq,   # 7  -> RR_thigh_joint to RR_thigh -> 7
            msg.motor_state[5].dq,   # 8  -> FL_calf_joint  to FL_calf  -> 5
            msg.motor_state[2].dq,   # 9  -> FR_calf_joint  to FR_calf  -> 2
            msg.motor_state[11].dq,  # 10 -> RL_calf_joint  to RL_calf  -> 11
            msg.motor_state[8].dq,   # 11 -> RR_calf_joint  to RR_calf  -> 8
        ])
    
    def cmd_vel_callback(self, msg):
        # command velocity
        self.cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        if self.cmd_vel is None or all(abs(v) < 0.1 for v in self.cmd_vel):
            self.cmd_vel = np.array([0.0, 0.0, 0.0])

    def generate_actions(self):
        # Check if model is loaded
        if self.ort_session is None:
            self.get_logger().warn("ONNX model not loaded, skipping inference")
            return
            
        # build observations
        obs = np.zeros(45)  
        obs[0:3] = self.ang_speed
        obs[3:6] = self.proj_gravity
        obs[6:9] = self.cmd_vel
        obs[9:21] = self.q
        obs[21:33] = self.dq
        obs[33:45] = self.raw_action # will be last_action
        input_obs = obs.astype(np.float32).reshape(1, -1)

        # run inference
        try:
            ort_inputs = {self.ort_session.get_inputs()[0].name: input_obs}
            ort_outs = self.ort_session.run(None, ort_inputs)
            self.raw_action = ort_outs[0].flatten()
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.raw_action = np.zeros(12)

        # accounting for offset and scale from isaac lab
        self.processed_actions = (self.raw_action * self.scale_factor + self.q_defaults).tolist()
        self.processed_actions_ordered = [
            self.processed_actions[1],  # 1  -> FR_hip_joint   to FR_hip   -> 0
            self.processed_actions[5],  # 5  -> FR_thigh_joint to FR_thigh -> 1
            self.processed_actions[9],  # 9  -> FR_calf_joint  to FR_calf  -> 2
            self.processed_actions[0],  # 0  -> FL_hip_joint   to FL_hip   -> 3
            self.processed_actions[4],  # 4  -> FL_thigh_joint to FL_thigh -> 4
            self.processed_actions[8],  # 8  -> FL_calf_joint  to FL_calf  -> 5
            self.processed_actions[3],  # 3  -> RR_hip_joint   to RR_hip   -> 6
            self.processed_actions[7],  # 7  -> RR_thigh_joint to RR_thigh -> 7
            self.processed_actions[11], # 11 -> RR_calf_joint  to RR_calf  -> 8
            self.processed_actions[2],  # 2  -> RL_hip_joint   to RL_hip   -> 9
            self.processed_actions[6],  # 6  -> RL_thigh_joint to RL_thigh -> 10
            self.processed_actions[10]  # 10 -> RL_calf_joint  to RL_calf  -> 11
        ]

        # publish action message
        action_msg = Float32MultiArray()
        action_msg.data = self.processed_actions_ordered
        self.publisher.publish(action_msg)

    def body_projected_gravity(self, quat):
        g_norm_world=np.array([0.0, 0.0, -1.0])
        rot = Rot.from_quat(quat, scalar_first=True)
        return rot.as_matrix() @ g_norm_world

    def load_onnx_model(self, policy_name):
        share_dir = get_package_share_directory('blind_locomotion')
        model_path = os.path.join(share_dir, 'models', f'{policy_name}.onnx')
        self.get_logger().info(f"Model path: {model_path}")
        try:
            self.ort_session = ort.InferenceSession(model_path)
            self.get_logger().info(f"Successfully loaded ONNX model: {policy_name}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load ONNX model: {e}")
            self.ort_session = None   


def main(args=None):
    rclpy.init(args=args)
    node = RLActionsNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
