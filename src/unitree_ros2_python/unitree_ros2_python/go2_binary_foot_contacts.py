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
            self.lowstate_callback,  # Correct callback name
            10
        )
        self.prev_time = self.get_clock().now()
        self.binary_foot_contacts = np.zeros(4)  # low state foot force data

    def lowstate_callback(self, msg):
        # Extract foot contact data from low state
        self.foot_forces = np.array([
            msg.foot_force[1],
            msg.foot_force[0],
            msg.foot_force[3],
            msg.foot_force[2]
        ])

        # Create binary contacts - needs to be edited
        for index, force in enumerate(self.foot_forces):
            self.binary_foot_contacts[index] = float(force > 40.0)

        binary_foot_contacts_msg = Float32MultiArray()
        binary_foot_contacts_msg.data = self.binary_foot_contacts
        self.publisher.publish(binary_foot_contacts_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BinaryFootContactsNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
