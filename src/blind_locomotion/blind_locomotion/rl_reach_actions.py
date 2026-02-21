#!/usr/bin/env python3
"""
RL Reaching/Standing Policy Inference Node

Observation space (53-dim):
  [0:3]   base_lin_vel      - Linear velocity (zeros, no estimator)
  [3:6]   base_ang_vel      - Angular velocity from IMU gyroscope
  [6:7]   base_height       - Base height (constant 0.25m)
  [7:10]  projected_gravity - Gravity vector in body frame (direct convention)
  [10:17] pose_command      - Target pose: position(3) + quaternion(4)
  [17:29] joint_pos         - Joint positions (Isaac Lab order, offset by defaults)
  [29:41] joint_vel         - Joint velocities (Isaac Lab order)
  [41:53] actions           - Last action output

Action space (12-dim):
  Raw actions in Isaac Lab joint order, scaled and offset before publishing
"""

import numpy as np
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState

from blind_locomotion.msg import JointPositionCommand

from blind_locomotion.gravity_utils import projected_gravity_direct
from blind_locomotion.joint_reorder import (
    actions_to_unitree_order,
    read_joint_positions_isaac,
    read_joint_velocities_isaac,
)
from blind_locomotion.onnx_loader import load_onnx_policy


class RLReachActionsNode(Node):
    def __init__(self):
        super().__init__('rl_reach_actions_publisher')

        # Parameters.
        self.declare_parameter('policy_name', 'SimplePolicy')
        self.declare_parameter('policy_frequency', 25)
        self.declare_parameter('scale_factor', 0.25)
        self.declare_parameter('default_hip_q', 0.0)
        self.declare_parameter('default_thigh_q', 0.8)
        self.declare_parameter('default_calf_q', -1.5)
        self.declare_parameter('base_height', 0.25)

        policy_name = self.get_parameter('policy_name').get_parameter_value().string_value
        policy_frequency = self.get_parameter('policy_frequency').get_parameter_value().integer_value
        scale_factor = self.get_parameter('scale_factor').get_parameter_value().double_value
        default_hip_q = self.get_parameter('default_hip_q').get_parameter_value().double_value
        default_thigh_q = self.get_parameter('default_thigh_q').get_parameter_value().double_value
        default_calf_q = self.get_parameter('default_calf_q').get_parameter_value().double_value
        self.base_height = self.get_parameter('base_height').get_parameter_value().double_value

        # Publisher.
        self.publisher = self.create_publisher(JointPositionCommand, 'actions', 10)

        # Subscriber.
        self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)

        # Load ONNX policy.
        self.ort_session = load_onnx_policy(
            self.get_logger(), policy_name,
            expected_input_dim=53, expected_output_dim=12,
        )
        self.timer = self.create_timer(1.0 / policy_frequency, self.generate_actions)

        # Isaac Lab constants.
        self.scale_factor = scale_factor
        self.q_defaults = np.concatenate((
            np.ones(4) * default_hip_q,
            np.ones(4) * default_thigh_q,
            np.ones(4) * default_calf_q,
        ), axis=0).astype(np.float32)

        # Fixed observation components.
        self.base_lin_vel = np.zeros(3, dtype=np.float32)
        self.pose_command = np.array(
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float32,
        )

        # Dynamic observation components (updated from lowstate).
        self.ang_speed = np.zeros(3, dtype=np.float32)
        self.proj_gravity = np.zeros(3, dtype=np.float32)
        self.q = np.zeros(12, dtype=np.float32)
        self.dq = np.zeros(12, dtype=np.float32)
        self.raw_action = np.zeros(12, dtype=np.float32)

        self.get_logger().info(f'Reach policy node started with {policy_name}')

    def lowstate_callback(self, msg):
        self.ang_speed = np.array(msg.imu_state.gyroscope[0:3], dtype=np.float32)

        quat_wxyz = np.array(msg.imu_state.quaternion[0:4], dtype=np.float32)
        self.proj_gravity = projected_gravity_direct(quat_wxyz)

        self.q = read_joint_positions_isaac(msg.motor_state) - self.q_defaults
        self.dq = read_joint_velocities_isaac(msg.motor_state)

    def generate_actions(self):
        if self.ort_session is None:
            self.get_logger().warn('ONNX model not loaded, skipping inference')
            return

        # Build 53-dim observation.
        obs = np.zeros(53, dtype=np.float32)
        obs[0:3]   = self.base_lin_vel
        obs[3:6]   = self.ang_speed
        obs[6]     = self.base_height
        obs[7:10]  = self.proj_gravity
        obs[10:17] = self.pose_command
        obs[17:29] = self.q
        obs[29:41] = self.dq
        obs[41:53] = self.raw_action

        input_obs = obs.reshape(1, -1)

        try:
            ort_inputs = {self.ort_session.get_inputs()[0].name: input_obs}
            ort_outs = self.ort_session.run(None, ort_inputs)
            self.raw_action = ort_outs[0].flatten().astype(np.float32)
        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
            self.raw_action = np.zeros(12, dtype=np.float32)

        # Apply scale and offset, then reorder to Unitree joint order.
        processed = self.raw_action * self.scale_factor + self.q_defaults
        unitree_order = actions_to_unitree_order(processed)

        action_msg = JointPositionCommand()
        action_msg.positions = unitree_order
        self.publisher.publish(action_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RLReachActionsNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
