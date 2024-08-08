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
        self.up = 0.0  # UP button state
        self.down = 0.0  # DOWN button state
        self.start = 0.0  # START button state
        self.select = 0.0  # SELECT button state
        self.a = 0.0  # A button state
        self.b = 0.0  # B button
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

        self.up = 1.0 if msg.keys == 4096 else 0.0
        self.down = 1.0 if msg.keys == 16384 else 0.0
        self.start = 1.0 if msg.keys == 4 else 0.0
        self.select = 1.0 if msg.keys == 8 else 0.0
        self.a = 1.0 if msg.keys == 256 else 0.0
        self.b = 1.0 if msg.keys == 512 else 0.0

        # Publishing the array of button states
        button_msg = Float32MultiArray()
        button_msg.data = [self.up, 
                           self.down, 
                           self.start,
                           self.select,
                           self.a,
                           self.b]

        # Publish the Twist message
        self.vel_publisher.publish(self.twist)

        # Publish the buttons message
        self.buttons_publisher.publish(button_msg)

        # Update the last message time
        self.last_msg_time = self.get_clock().now()

    def check_timeout(self):
        if self.get_clock().now() - self.last_msg_time > self.timeout_duration:
            # Publish zero velocities if timeout occurs
            zero_twist = Twist()
            self.vel_publisher.publish(zero_twist)

            # Publish zero button states
            zero_button_msg = Float32MultiArray()
            zero_button_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.buttons_publisher.publish(zero_button_msg)


def main(args=None):
    rclpy.init()
    node = WirelessControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
