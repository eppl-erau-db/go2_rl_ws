#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_go.msg import WirelessController


class WirelessControl(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.wireless_controller_callback,
            10)
        self.twist = Twist()
        self.speed = 1.0  # Adjust speed as needed (up to 1 m/s)
        self.turn = 1.0  # Adjust turn rate as needed (up to 1 rad/s)
        self.get_logger().info("Wireless controller teleop started. Use joystick for motion.")
        self.active_keys = set()

    def wireless_controller_callback(self, msg):
        self.twist = Twist()
        self.twist.linear.x = msg.lx * self.speed
        self.twist.linear.y = msg.ly * self.speed
        self.twist.angular.z = msg.rx * self.turn
        # Publish the Twist message
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = WirelessControl()
    rclpy.spin(node)
    # Clean up when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
