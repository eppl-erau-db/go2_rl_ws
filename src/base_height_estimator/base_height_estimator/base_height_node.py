#!/usr/bin/env python3
"""ROS 2 node that estimates base height from joint FK + foot contact forces."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float32
from unitree_go.msg import LowState

from base_height_estimator.go2_fk import fk

# FK leg_id -> (motor indices, foot_force index)
LEG_CONFIG = {
    0: {'motors': (3, 4, 5), 'force_idx': 1},    # FL
    1: {'motors': (0, 1, 2), 'force_idx': 0},    # FR
    2: {'motors': (9, 10, 11), 'force_idx': 3},  # RL
    3: {'motors': (6, 7, 8), 'force_idx': 2},    # RR
}


class BaseHeightEstimator(Node):
    def __init__(self):
        super().__init__('base_height_estimator')

        self.declare_parameter('force_threshold', 20.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('filter_alpha', 0.2)

        self.force_threshold = self.get_parameter('force_threshold').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.filter_alpha = self.get_parameter('filter_alpha').value

        self.filtered_height = None
        self.min_publish_period_sec = 1.0 / publish_rate_hz
        self.last_publish_time = self.get_clock().now()

        self.publisher = self.create_publisher(Float32, '/base_height', 10)
        self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)

        self.get_logger().info(
            f'Base height estimator: force_thresh={self.force_threshold:.1f}N '
            f'rate={publish_rate_hz:.0f}Hz alpha={self.filter_alpha:.2f}'
        )

    def lowstate_callback(self, msg):
        # Rate-limit publishing.
        now = self.get_clock().now()
        dt = (now - self.last_publish_time).nanoseconds * 1e-9
        if dt < self.min_publish_period_sec:
            return

        # IMU quaternion: LowState gives [w,x,y,z], scipy wants [x,y,z,w].
        qw, qx, qy, qz = msg.imu_state.quaternion[0:4]
        R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

        heights = []
        for leg_id, cfg in LEG_CONFIG.items():
            foot_force = msg.foot_force[cfg['force_idx']]
            if foot_force < self.force_threshold:
                continue

            q = np.array([
                msg.motor_state[cfg['motors'][0]].q,
                msg.motor_state[cfg['motors'][1]].q,
                msg.motor_state[cfg['motors'][2]].q,
            ])

            p_foot = fk(leg_id, q)
            foot_z_world = R[2, :] @ p_foot
            heights.append(-foot_z_world)

        if not heights:
            return

        raw_height = float(np.mean(heights))

        if self.filtered_height is None:
            self.filtered_height = raw_height
        else:
            self.filtered_height += self.filter_alpha * (raw_height - self.filtered_height)

        out = Float32()
        out.data = self.filtered_height
        self.publisher.publish(out)
        self.last_publish_time = now


def _spin_until_shutdown(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BaseHeightEstimator()
    _spin_until_shutdown(node)


if __name__ == '__main__':
    main()
