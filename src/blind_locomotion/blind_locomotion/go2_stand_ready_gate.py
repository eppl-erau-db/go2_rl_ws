#!/usr/bin/env python3
# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
"""
Block until the low-level controller reports a confirmed stand, then exit.

Used by launch files to sequence actions that must only run once the robot is
actually standing (e.g. starting leg odometry, which accumulates drift if it is
started while the robot is lying down). Exits 0 on confirmation, 1 on timeout.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class StandReadyGate(Node):
    def __init__(self):
        super().__init__('go2_stand_ready_gate')

        self.declare_parameter('stand_ready_topic', 'stand_ready')
        self.declare_parameter('required_consecutive_samples', 10)
        self.declare_parameter('settle_sec', 1.0)
        self.declare_parameter('timeout_sec', 30.0)

        self.stand_ready_topic = str(self.get_parameter('stand_ready_topic').value)
        self.required_consecutive_samples = max(
            1, int(self.get_parameter('required_consecutive_samples').value)
        )
        self.settle_sec = max(0.0, float(self.get_parameter('settle_sec').value))
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self._consecutive_ready = 0
        self._received_any = False
        self.confirmed = False

        self.create_subscription(Bool, self.stand_ready_topic, self._callback, 10)

        self.get_logger().info(
            'Waiting for %d consecutive stand-ready samples on %s (timeout %.1fs)'
            % (self.required_consecutive_samples, self.stand_ready_topic, self.timeout_sec)
        )

    def _callback(self, msg):
        if not self._received_any:
            self._received_any = True
            self.get_logger().info(
                'Received first stand-ready sample: %s' % ('true' if msg.data else 'false')
            )
        if msg.data:
            self._consecutive_ready += 1
        else:
            self._consecutive_ready = 0
        if self._consecutive_ready >= self.required_consecutive_samples:
            self.confirmed = True

    def wait(self):
        deadline = None
        if self.timeout_sec > 0.0:
            deadline = time.monotonic() + self.timeout_sec

        while rclpy.ok() and not self.confirmed:
            if deadline is not None and time.monotonic() >= deadline:
                if not self._received_any:
                    self.get_logger().error(
                        'Timed out: no messages on %s. Is go2_controller_node running?'
                        % self.stand_ready_topic
                    )
                else:
                    self.get_logger().error(
                        'Timed out waiting for a confirmed stand on %s'
                        % self.stand_ready_topic
                    )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.confirmed:
            return False

        self.get_logger().info('Stand confirmed')
        if self.settle_sec > 0.0:
            self.get_logger().info(
                'Waiting %.1fs for the robot to settle before releasing the gate'
                % self.settle_sec
            )
            time.sleep(self.settle_sec)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = StandReadyGate()
    ok = False
    try:
        ok = node.wait()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
