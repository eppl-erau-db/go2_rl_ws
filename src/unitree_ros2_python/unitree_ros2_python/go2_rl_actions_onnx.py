#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import array
import onnxruntime as ort
import os
from ament_index_python.packages import get_package_share_directory


class Go2_RL_Actions(Node):
    def __init__(self):
        super().__init__("action_publisher")
        # Creating publisher to actions topic
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'actions',
            10)
        
        # Creating subscriptions to obs topics
        self.create_subscription(
            Float32MultiArray,
            'base_ang_vel',
            self.base_ang_vel_callback,
            10)
        self.create_subscription(
            Float32MultiArray,
            'projected_gravity',
            self.projected_gravity_callback,
            10)
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.create_subscription(
            Float32MultiArray,
            'joint_pos_vel',
            self.joint_pos_vel_callback,
            10)

        # Initializing variables
        self.init_raw_action = None
        self.raw_actions = None
        self.processed_actions = None
        self.base_ang_vel = np.zeros(3)
        self.projected_gravity = np.zeros(3)  
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.joint_pos_vel = np.zeros(24)
        self.joint_pos_init = None

        # Defaults and scale from isaac lab:
        self.motor_qs_defaults = np.array([
            0.0,
            -0.0,
            0.0,
            -0.0,
            1.1,
            1.1,
            1.1,
            1.1,
            -1.8,
            -1.8,
            -1.8,
            -1.8,
        ], dtype=np.float32)
        self.scale_factor = 0.25  # Can be found in isaac lab repo

        # Finding and loading the ONNX model - can be changed to desired model
        share_dir = get_package_share_directory('unitree_ros2_python')
        # pronking gait - do not use v3 or v5 currently
        model_path = os.path.join(share_dir, 'models', 'flat_policy_v5.onnx') 
        self.get_logger().info(f"Model path: {model_path}")
        self.load_onnx_model(model_path)

        # Create a timer to generate actions every 20 milliseconds (50 Hz)
        self.timer_period = 0.02  # 20 milliseconds
        self.timer = self.create_timer(
            self.timer_period,
            self.generate_actions)

    def load_onnx_model(self, model_path):
        try:
            self.ort_session = ort.InferenceSession(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")

    def base_ang_vel_callback(self, msg):
        self.base_ang_vel = np.array(msg.data, dtype=np.float32)

    def projected_gravity_callback(self, msg):
        self.projected_gravity = np.array(msg.data, dtype=np.float32)

    def cmd_vel_callback(self, msg):
        # Converting twist message to array for concatenation
        self.cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]

    def joint_pos_vel_callback(self, msg):
        self.joint_pos_vel = np.array(msg.data, dtype=np.float32)
        # The first 12 elements are joint positions
        self.joint_pos_init = self.joint_pos_vel[:12]
        # Getting initial raw action
        self.init_raw_action = (self.joint_pos_init - self.motor_qs_defaults) / self.scale_factor

    def generate_actions(self):
        # Check for valid inputs
        if self.cmd_vel is None or all(abs(v) < 0.1 for v in self.cmd_vel):
            self.cmd_vel = [0.0, 0.0, 0.0]

        # Creating obs vector (without last action)
        obs = np.concatenate((
            self.base_ang_vel,
            self.projected_gravity,
            self.cmd_vel,
            self.joint_pos_vel
        ))

        # Adding last action or first raw action for obs
        if self.raw_actions is not None:
            obs = np.concatenate((obs, self.raw_actions))
        else:
            obs = np.concatenate((obs, self.init_raw_action))

        # Make obs into np array & reshape for the batch size
        obs = obs.astype(np.float32).reshape(1, -1)

        # Run inference
        try:
            ort_inputs = {self.ort_session.get_inputs()[0].name: obs}
            ort_outs = self.ort_session.run(None, ort_inputs)
            self.raw_actions = ort_outs[0].flatten()
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.raw_actions = np.zeros_like(self.init_raw_action)

        # Accounting for offset and scale (Isaac Lab)
        self.processed_actions = (self.raw_actions * self.scale_factor + self.motor_qs_defaults).tolist()
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

        # Motor limits
        self.motor_limits_ordered = [
            [-0.837, 0.837],  # Front Hip
            [-3.490, 1.570],  # Front Thigh
            [-2.720, 0.837],  # Front Calf
            [-0.837, 0.837],  # Front Hip
            [-3.490, 1.570],  # Front Thigh
            [-2.720, 0.837],  # Front Calf
            [-0.837, 0.837],  # Rear Hip
            [-4.530, 1.570],  # Rear Thigh
            [-2.720, 0.837],  # Rear Calf
            [-0.837, 0.837],  # Rear Hip
            [-4.530, 1.570],  # Rear Thigh
            [-2.720, 0.837]   # Rear Calf
        ]

        # Clipping actions by a scale of the motor limits:
        clipped_actions_ordered = [0]*12
        for i, action in enumerate(self.processed_actions_ordered):
            min_limit, max_limit = self.motor_limits_ordered[i]

            # Applying scale factor
            min_limit = min_limit * 0.95
            max_limit = max_limit * 0.95
            clipped_actions_ordered[i] = max(min(action, max_limit), min_limit)

        # Publishing action messages
        action_msg = Float32MultiArray()
        action_msg.data = clipped_actions_ordered
        self.publisher.publish(action_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2_RL_Actions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
