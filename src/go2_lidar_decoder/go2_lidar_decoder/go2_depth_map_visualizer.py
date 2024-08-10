#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt

class DepthMapSubscriber(Node):

    def __init__(self):
        super().__init__('depth_map_subscriber')

        # Initialize parameters
        self.declare_parameter('length', 1.0)
        self.declare_parameter('width', 1.0)
        self.declare_parameter('resolution', 0.1)

        self.length = self.get_parameter('length').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value

        self.length_cells = int(self.length / self.resolution)
        self.width_cells = int(self.width / self.resolution)

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'depth_map',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Convert the 1D list to a 2D numpy array using the correct dimensions
        try:
            data = np.array(msg.data).reshape((self.length_cells, self.width_cells))
        except ValueError as e:
            self.get_logger().error(f'Error reshaping data: {e}')
            return

        # Visualize the depth map
        self.visualize_depth_map(data)

    def visualize_depth_map(self, depth_map):
        plt.imshow(depth_map, cmap='gray', origin='lower')
        plt.colorbar(label='Depth (m)')
        plt.title('Depth Map')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    depth_map_subscriber = DepthMapSubscriber()
    rclpy.spin(depth_map_subscriber)
    depth_map_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
