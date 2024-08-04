import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
import tf_transformations

class LidarToHeightMap(Node):
    def __init__(self):
        super().__init__('lidar_to_heightmap')

        # Declare and get parameters
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('length', 1.0)  # Length in meters
        self.declare_parameter('width', 1.0)   # Width in meters
        self.declare_parameter('min_height', -1.0)
        self.declare_parameter('max_height', 1.0)
        self.declare_parameter('offset', 0.31)

        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.length = self.get_parameter('length').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().double_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.offset = self.get_parameter('offset').get_parameter_value().double_value

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

        # Reset the height map
        self.height_map.fill(self.min_height)

        for point in points_base:
            x, y, z = point
            i = int((x + (self.length / 2)) / self.resolution)
            j = int((y + (self.width / 2)) / self.resolution)

            if 0 <= i < self.length_cells and 0 <= j < self.width_cells:
                self.height_map[i, j] = max(self.height_map[i, j], z)


        # Interpolate missing values by averaging neighboring cells
        for i in range(self.height_map.shape[0]):
            for j in range(self.height_map.shape[1]):
                cell_value = self.height_map[i, j]
                if cell_value == self.min_height or cell_value >= self.max_height:
                    # Collect neighboring cell values
                    neighbors = []
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            ni = i + di
                            nj = j + dj
                            if 0 <= ni < self.height_map.shape[0] and 0 <= nj < self.height_map.shape[1]:
                                neighbor_value = self.height_map[ni, nj]
                                if neighbor_value != self.min_height:
                                    neighbors.append(neighbor_value)
                    
                    if neighbors:
                        # Calculate the mean of the valid neighbors
                        self.height_map[i, j] = np.mean(neighbors)

        # Publish the height map as Float32MultiArray
        self.publish_height_map()

    def publish_height_map(self):
        # Flip the height map:
        self.height_map = np.flip(self.height_map, axis=1) 

        # Create and publish message
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
