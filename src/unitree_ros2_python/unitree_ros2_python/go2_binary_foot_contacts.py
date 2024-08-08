#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState
import numpy as np


class BinaryFootContactsNode(Node):
    def __init__(self):
        super().__init__("binary_foot_contacts_publisher")
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'binary_foot_contacts',
            10
        )
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate', 
            self.imu_callback,
            10
        )
        self.prev_time = self.get_clock().now()
        self.binary_foot_contacts = np.zeros(4)  # Gyroscope readings

    def lowstate_callback(self, msg):
        # Extract gyroscope readings from IMUState message
        self.foot_forces = np.array([
            msg.foot_force_est[0],
            msg.foot_force_est[1],
            msg.foot_force_est[2],
            msg.foot_force_est[3]
        ])

        # Update gyroscope data
        for index, force in self.foot_forces:
            self.binary_foot_contacts[index] = float(force > 1.0)

        # Publish gyroscope readings
        binary_foot_contacts_msg = Float32MultiArray()
        binary_foot_contacts_msg.data = self.binary_foot_contacts.tolist()
        self.publisher.publish(binary_foot_contacts_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BinaryFootContactsNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()