#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
import tf_transformations

class LidarToDepthMap(Node):
    def __init__(self):
        super().__init__('lidar_to_depthmap')

        # Declare and get parameters
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('length', 1.0)  # Length in meters
        self.declare_parameter('width', 1.0)   # Width in meters
        self.declare_parameter('min_depth', 0.0) # Min depth value (if needed)
        self.declare_parameter('max_depth', 10.0) # Max depth value (if needed)
        self.declare_parameter('offset', -0.35)

        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.length = self.get_parameter('length').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().double_value
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value
        self.offset = self.get_parameter('offset').get_parameter_value().double_value

        # Calculate the number of cells
        self.length_cells = int(self.length / self.resolution)
        self.width_cells = int(self.width / self.resolution)

        # Initialize the depth map
        self.depth_map = np.full((self.length_cells, self.width_cells), self.max_depth, dtype=np.float32)

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
            '/depth_map',
            10
        )

    def lidar_callback(self, msg):
        # Extract the points from the PointCloud2 message
        point_cloud = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Convert to a list of tuples
        points_list = list(point_cloud)

        # Extract fields and convert to NumPy array
        x = np.array([point[0] for point in points_list], dtype=np.float32)
        y = np.array([point[1] for point in points_list], dtype=np.float32)
        z = np.array([point[2] for point in points_list], dtype=np.float32)

        # Stack x, y, z to form the points array
        points_array = np.column_stack((x, y, z - self.offset))

        # Create homogeneous coordinates
        points_homogeneous = np.column_stack((points_array, np.ones(x.shape)))

        # Define the rotation about the y-axis in radians
        angle_y = 2.8782  # Rotation about y-axis in radians

        # Compute the rotation matrix around the y-axis
        rotation_matrix_1 = tf_transformations.euler_matrix(0.0, angle_y, 0.0)

        # Transform points from LiDAR frame to base frame (only applying rotation)
        points_base = (rotation_matrix_1[:3, :3] @ points_homogeneous[:, :3].T).T

        # Reset the depth map
        self.depth_map.fill(self.max_depth)

        for point in points_base:
            x, y, z = point
            i = int((y + (self.length / 2)) / self.resolution)
            j = int((z + (self.width / 2)) / self.resolution)

            if 0 <= i < self.length_cells and 0 <= j < self.width_cells:
                self.depth_map[i, j] = min(self.depth_map[i, j], x)
                self.depth_map[i, j] = max(self.depth_map[i, j], self.min_depth)

        # Interpolate missing values by averaging neighboring cells
        for i in range(self.depth_map.shape[0]):
            for j in range(self.depth_map.shape[1]):
                cell_value = self.depth_map[i, j]
                if cell_value == self.max_depth:
                    # Collect neighboring cell values
                    neighbors = []
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            ni = i + di
                            nj = j + dj
                            if 0 <= ni < self.depth_map.shape[0] and 0 <= nj < self.depth_map.shape[1]:
                                neighbor_value = self.depth_map[ni, nj]
                                if neighbor_value != self.max_depth:
                                    neighbors.append(neighbor_value)
                    
                    if neighbors:
                        # Calculate the mean of the valid neighbors
                        self.depth_map[i, j] = np.mean(neighbors)

        # Publish the depth map as Float32MultiArray
        self.publish_depth_map()

    def publish_depth_map(self):
        # Optionally flip the depth map if needed:
        self.depth_map = np.rot90(self.depth_map, k=1) 

        # Create and publish message
        msg = Float32MultiArray()
        msg.data = self.depth_map.flatten().tolist()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarToDepthMap()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
