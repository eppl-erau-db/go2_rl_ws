#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState, IMUState
import numpy as np


class BaseVelocityNode(Node):
    def __init__(self):
        super().__init__("base_ang_vel_publisher")
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'base_ang_vel',
            10
        )
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate', 
            self.imu_callback,
            10
        )
        self.prev_time = self.get_clock().now()
        self.gyroscope_data = np.zeros(3)  # Gyroscope readings

    def imu_callback(self, msg):
        # Extract gyroscope readings from IMUState message
        gyroscope = np.array([
            msg.imu_state.gyroscope[0],
            msg.imu_state.gyroscope[1],
            msg.imu_state.gyroscope[2]
        ])

        # Update gyroscope data
        self.gyroscope_data = gyroscope

        # Publish gyroscope readings
        velocity_msg = Float32MultiArray()
        velocity_msg.data = self.gyroscope_data.tolist()
        self.publisher.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseVelocityNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
