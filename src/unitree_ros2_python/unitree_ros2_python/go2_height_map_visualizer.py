#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt


class HeightMapSubscriber(Node):

    def __init__(self):
        super().__init__('height_map_subscriber')

        # Initialize parameters
        self.declare_parameter('length', 1.6)
        self.declare_parameter('width', 1.0)
        self.declare_parameter('resolution', 0.1)

        self.length = self.get_parameter('length').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value

        self.length_cells = int(self.length / self.resolution)
        self.width_cells = int(self.width / self.resolution)

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'height_map',
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

        # Visualize the height map
        self.visualize_height_map(data)

    def visualize_height_map(self, height_map):
        plt.imshow(height_map, cmap='terrain', origin='lower')
        plt.colorbar(label='Height (m)')
        plt.title('Height Map')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    height_map_subscriber = HeightMapSubscriber()
    rclpy.spin(height_map_subscriber)
    height_map_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()