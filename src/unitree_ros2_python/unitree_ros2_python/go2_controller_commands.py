#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_go.msg import WirelessController
from rclpy.duration import Duration
from std_msgs.msg import Float32MultiArray


class WirelessControl(Node):
    def __init__(self):
        super().__init__('wireless_control')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.buttons_publisher = self.create_publisher(
            Float32MultiArray,
            'buttons',
            10)
        self.subscription = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.wireless_controller_callback,
            10)
        self.twist = Twist()
        self.buttons = [0, 0, 0]  # up, down, start
        self.speed = 1.0  # Max lin speed, m/s
        self.turn = -1.0  # Max ang speed, rad/s (sign convention switch)
        self.last_msg_time = self.get_clock().now()
        self.timeout_duration = Duration(seconds=0.5)  # Timeout duration
        self.get_logger().info(
            "Wireless controller control started. Use joystick for motion."
            )
        # Check timeout
        self.timer = self.create_timer(0.1, self.check_timeout)

    def wireless_controller_callback(self, msg):
        self.twist = Twist()
        self.twist.linear.x = msg.ly * self.speed
        self.twist.linear.y = msg.lx * -self.speed  # (sign convention switch)
        self.twist.angular.z = msg.rx * self.turn
        self.data = [msg.keys.up, msg.keys.down, msg.keys.start]
        # Publish the Twist message
        self.vel_publisher.publish(self.twist)

        # Publish the buttons message
        self.buttons_publisher.publish(self.data.tolist())

        # Update the last message time
        self.last_msg_time = self.get_clock().now()

    def check_timeout(self):
        if self.get_clock().now() - self.last_msg_time > self.timeout_duration:
            # Publish zero velocities if timeout occurs
            zero_twist = Twist()
            self.vel_publisher.publish(zero_twist)


def main(args=None):
    rclpy.init(args=args)
    node = WirelessControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
