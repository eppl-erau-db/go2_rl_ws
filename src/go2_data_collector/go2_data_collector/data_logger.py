#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, LowCmd, SportModeState
import numpy as np
import os
from collections import deque


class DataLogger(Node):
    def __init__(self):
        super().__init__('motor_data_logger')

        # Subscriptions
        self.sub_lowstate = self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)
        self.sub_lowcmd = self.create_subscription(LowCmd, '/lowcmd', self.lowcmd_callback, 10)


        # Data buffers
        self.last_dq = None
        self.last_time = None
        self.dataset = deque(maxlen=100000)  # adjustable
        self.data_save_path = 'motor_dataset.npz'

        # Message cache
        self.lowstate = None
        self.lowcmd = None
        self.motor_modes = [0] * 12

        # IsaacLab reordering (index: new → old)
        self.joint_map = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]

    def lowcmd_callback(self, msg: LowCmd):
        self.lowcmd = msg

    def lowstate_callback(self, msg: LowState):
        self.lowstate = msg

        # Check motor control modes
        modes = [m.mode for m in msg.motor_state]
        if any(mode != 0x01 for mode in modes[:12]):
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = now
            self.last_dq = np.zeros(12)
            return

        dt = now - self.last_time
        if dt <= 0.0:
            return

        q     = np.array([msg.motor_state[i].q       for i in self.joint_map])
        dq    = np.array([msg.motor_state[i].dq      for i in self.joint_map])
        tau   = np.array([msg.motor_state[i].tau_est for i in self.joint_map])
        q_des = (np.array([self.lowcmd.motor_cmd[i].q for i in self.joint_map])
                if self.lowcmd else np.zeros(12))

        ddq = (dq - self.last_dq) / dt
        x = np.concatenate([q, dq, ddq, q_des])
        y = tau

        self.dataset.append((x.astype(np.float32), y.astype(np.float32)))
        self.last_dq = dq
        self.last_time = now

        if len(self.dataset) % 1000 == 0:
            self.save_dataset()


    def save_dataset(self):
        if len(self.dataset) == 0:
            return
        X, Y = zip(*self.dataset)
        np.savez_compressed(self.data_save_path, X=np.array(X), Y=np.array(Y))
        self.get_logger().info(f"Saved dataset with {len(self.dataset)} samples to {self.data_save_path}")

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
