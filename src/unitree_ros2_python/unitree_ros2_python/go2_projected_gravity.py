#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState, IMUState
import numpy as np
from scipy.spatial.transform import Rotation as R


class ProjectedGravityNode(Node):
    def __init__(self):
        super().__init__("projected_gravity_publisher")
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'projected_gravity',
            10
        )
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        imu_quaternion = np.array([
            msg.imu_state.quaternion[0],
            msg.imu_state.quaternion[1],
            msg.imu_state.quaternion[2],
            msg.imu_state.quaternion[3]
        ])

        # Calculate projected gravity vector
        gravity_proj = self.projected_gravity_vector(imu_quaternion)

        # Publish as Float32 Array
        gravity_msg = Float32MultiArray()
        gravity_msg.data = gravity_proj.tolist()
        self.publisher.publish(gravity_msg)

    def projected_gravity_vector(self, imu_quaternion):
        # Use rotation from quaternion to find proj g
        rotation = R.from_quat(imu_quaternion)
        gravity_vec_w = np.array([0.0, 0.0, -1.0])  # Gravity vector in world
        gravity_proj = -1 * rotation.apply(gravity_vec_w)
        return gravity_proj


def main(args=None):
    rclpy.init(args=args)
    node = ProjectedGravityNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
