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
        self.publisher = self.create_publisher(Float32MultiArray, 'actions', 10)

        # Creating subscriptions to other topics
        self.create_subscription(Float32MultiArray, 'base_vel', self.base_vel_callback, 10)
        self.create_subscription(Float32MultiArray, 'projected_gravity', self.projected_gravity_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32MultiArray, 'joint_pos_vel', self.joint_pos_vel_callback, 10)

        # Initializing variables
        self.init_raw_action = None
        self.raw_actions = None
        self.processed_actions = None
        self.base_vel = None
        self.projected_gravity = None
        self.cmd_vel = None
        self.joint_pos_vel = None
        self.joint_pos_init = None

        # Defaults and scale from isaac lab:
        self.motor_qs_defaults = np.array([
            0.1,
            -0.1,
            0.1,
            -0.1,
            0.8,
            0.8,
            1.0,
            1.0,
            -1.5,
            -1.5,
            -1.5,
            -1.5,
        ], dtype=np.float32)
        self.scale_factor = 0.25

        # Finding and loading the ONNX model
        package_share_directory = get_package_share_directory('unitree_ros2_python')
        model_path = os.path.join(package_share_directory, 'models', 'flat_policy_v2.onnx')
        self.get_logger().info(f"Model path: {model_path}")
        self.load_onnx_model(model_path)

        # Create a timer to generate actions every 20 milliseconds (50 Hz)
        self.timer_period = 0.02  # 20 milliseconds
        self.timer = self.create_timer(self.timer_period, self.generate_actions)

    def load_onnx_model(self, model_path):
        self.ort_session = ort.InferenceSession(model_path)
        
    def base_vel_callback(self, msg):
        self.base_vel = msg.data

    def projected_gravity_callback(self, msg):
        self.projected_gravity = msg.data

    def cmd_vel_callback(self, msg):
        self.cmd_vel = array.array('f',
                                   [msg.linear.x,
                                    msg.linear.y,
                                    msg.angular.z])

    def joint_pos_vel_callback(self, msg):
        self.joint_pos_vel = msg.data
        self.joint_pos_init = msg.data[:12]  # Assuming the first 12 elements are joint positions
        self.init_raw_action = (self.joint_pos_init-self.motor_qs_defaults)/self.scale_factor

    def generate_actions(self):
        # Creating initial observations without last action:
        if self.cmd_vel is None:
            self.cmd_vel = array.array('f', [0.0, 0.0, 0.0])  # Default values

        obs = np.concatenate((self.base_vel, self.projected_gravity, self.cmd_vel, self.joint_pos_vel), axis=None)

        # Adding last action or first RAW action for obs
        if self.raw_actions is not None:
            obs = np.concatenate((obs, self.raw_actions), axis=None)
        else:
            obs = np.concatenate((obs, self.init_raw_action), axis=None)
        
        print(obs)

        # Make obs into np array
        obs = obs.astype(np.float32)
        
        # Reshape obs to add an extra dimension for the batch size
        obs = obs.reshape(1, -1)

        # Run inference 
        ort_inputs = {self.ort_session.get_inputs()[0].name: obs}
        ort_outs = self.ort_session.run(None, ort_inputs)
        self.raw_actions = ort_outs[0].flatten()
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

        action_msg = Float32MultiArray()
        action_msg.data = self.processed_actions_ordered
        self.publisher.publish(action_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2_RL_Actions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
