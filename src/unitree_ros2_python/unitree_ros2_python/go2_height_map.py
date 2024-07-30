#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray


class LidarToHeightMap(Node):
    def __init__(self):
        super().__init__('lidar_to_heightmap')

        # Declare and get parameters
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('length', 1.6)  # Length in meters
        self.declare_parameter('width', 1.0)   # Width in meters
        self.declare_parameter('min_height', -2.0)
        self.declare_parameter('max_height', 2.0)

        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.length = self.get_parameter('length').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().double_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value

        # Calculate the number of cells
        self.length_cells = int(self.length / self.resolution)
        self.width_cells = int(self.width / self.resolution)

        # Initialize the height map
        self.height_map = np.full((self.length_cells, self.width_cells), self.min_height, dtype=np.float32)

        # Subscribe to the PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.lidar_callback,
            10
        )

        # Create a publisher for the Float32MultiArray
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/height_map',
            10)

    def lidar_callback(self, msg):
        # Extract the points from the PointCloud2 message
        points = np.array([point for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])

        # Reset the height map
        self.height_map.fill(self.min_height)

        for point in points:
            x, y, z = point
            i = int((x + (self.length / 2)) / self.resolution)
            j = int((y + (self.width / 2)) / self.resolution)

            if 0 <= i < self.length_cells and 0 <= j < self.width_cells:
                self.height_map[i, j] = max(self.height_map[i, j], z-0.3)  # added offset

        # Publish the height map as Float32MultiArray
        self.publish_height_map()

    def publish_height_map(self):
        msg = Float32MultiArray()
        msg.data = self.height_map.flatten().tolist()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarToHeightMap()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
