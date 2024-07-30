#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from std_msgs.msg import Float32MultiArray
import numpy as np
import onnxruntime as ort
import os
from ament_index_python.packages import get_package_share_directory


class Go2_RL_Nav_Actions(Node):
    def __init__(self):
        super().__init__("nav_action_publisher")
        # Creating publisher to actions topic
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'nav_actions',
            10)
        # Creating subscriptions to obs topics
        self.create_subscription(
            SportModeState,
            'rt/sportmodestate',
            self.state_callback,
            10)
        self.create_subscription(
            Float32MultiArray,
            'projected_gravity',
            self.projected_gravity_callback,
            10)
        self.create_subscription(
            Float32MultiArray,
            'cmd_pose',
            self.cmd_pose_callback,
            10)

        # Initializing variables
        self.nav_actions = None
        self.processed_actions = None
        self.base_vel = None
        self.projected_gravity = None
        self.cmd_pose = [0.0, 0.0, 0.0, 0.0]

        # Defaults and scale from isaac lab:
        self.pose_cmd_defaults = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # Finding and loading the ONNX model - can be changed to desired model
        share_dir = get_package_share_directory('rl_navigation')
        model_path = os.path.join(share_dir, 'models', 'flat_nav_policy.onnx')
        self.get_logger().info(f"Model path: {model_path}")
        self.load_onnx_model(model_path)

        # Create a timer to generate actions every 20 milliseconds (50 Hz)
        self.timer_period = 0.02  # 20 milliseconds
        self.timer = self.create_timer(
            self.timer_period,
            self.generate_actions)

    def load_onnx_model(self, model_path):
        self.ort_session = ort.InferenceSession(model_path)

    def state_callback(self, msg):
        self.base_vel = np.array(msg.velocity, dtype=np.float32)

    def projected_gravity_callback(self, msg):
        self.projected_gravity = msg.data

    def cmd_pose_callback(self, msg):
        self.cmd_pose = msg.data

    def generate_actions(self):
        # Log the current state of the input components
        self.get_logger().info(f"Base velocity: {self.base_vel}")
        self.get_logger().info(f"Projected gravity: {self.projected_gravity}")
        self.get_logger().info(f"Cmd pose: {self.cmd_pose}")

        # Creating obs vector (without last action)
        obs = np.concatenate((self.base_vel,
                              self.projected_gravity,
                              self.cmd_pose), axis=None)

        # Log the obs vector and its size
        self.get_logger().info(f"Obs vector: {np.array2string(obs, precision=3, separator=', ')}")
        self.get_logger().info(f"Obs vector size: {obs.size}")

        # Make obs into np array & reshape for the batch size
        obs = obs.astype(np.float32)
        obs = obs.reshape(1, -1)

        # Run inference
        ort_inputs = {self.ort_session.get_inputs()[0].name: obs}
        ort_outs = self.ort_session.run(None, ort_inputs)
        self.nav_actions = ort_outs[0].flatten()

        # Publishing action messages
        action_msg = Float32MultiArray()
        action_msg.data = self.nav_actions.tolist()
        self.publisher.publish(action_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2_RL_Nav_Actions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
