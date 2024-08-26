#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np


class PoseCommand(Node):
    def __init__(self):
        super().__init__('pose_command')

        # Declare parameters
        self.declare_parameter('x_cmd', 0.0)
        self.declare_parameter('y_cmd', 0.0)
        self.declare_parameter('heading_cmd', 0.0)

        # Publisher
        self.pose_publisher = self.create_publisher(
            Float32MultiArray,
            'cmd_pose',
            10
        )

        # Initialize cmd pose.
        self.cmd_pose = np.array([0.0, 0.0, 0.305, 0.0], dtype=np.float32)

        # Create a timer to check input at regular intervals
        self.timer = self.create_timer(0.1, self.get_pose_command)

    def get_pose_command(self):
        self.cmd_pose[0] = self.get_parameter('x_cmd').get_parameter_value().double_value
        self.cmd_pose[1] = self.get_parameter('y_cmd').get_parameter_value().double_value
        self.cmd_pose[3] = self.get_parameter('heading_cmd').get_parameter_value().double_value

        # Publish the pose command
        msg = Float32MultiArray(data=self.cmd_pose)
        self.pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
