#!/usr/bin/env python3
"""
Dummy action publisher for testing stand/sit transitions.
Publishes standing position actions without any RL inference.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class TestActionPublisher(Node):
    def __init__(self):
        super().__init__('test_action_publisher')
        
        # Parameters
        self.declare_parameter('publish_frequency', 25.0)
        self.declare_parameter('verbose', True)
        
        freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        
        # Standing position for Go2 (Unitree joint order)
        # FR(hip,thigh,calf), FL(hip,thigh,calf), RR(hip,thigh,calf), RL(hip,thigh,calf)
        self.standing_actions = [
            0.0, 1.1, -1.8,   # FR
            0.0, 1.1, -1.8,   # FL
            0.0, 1.1, -1.8,   # RR
            0.0, 1.1, -1.8,   # RL
        ]
        
        # Publisher
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'actions',
            10
        )
        
        # Timer
        self.timer = self.create_timer(1.0 / freq, self.publish_actions)
        
        self.get_logger().info(f'Test action publisher started at {freq} Hz')
        self.get_logger().info(f'Publishing standing position: {self.standing_actions}')
        
    def publish_actions(self):
        msg = Float32MultiArray()
        msg.data = self.standing_actions
        self.publisher.publish(msg)
        
        if self.verbose:
            self.get_logger().debug('Published standing actions')


def main(args=None):
    rclpy.init(args=args)
    node = TestActionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
