#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from unitree_go.msg import WirelessController
from blind_locomotion.msg import Button


class WirelessControl(Node):
    def __init__(self):
        super().__init__('wireless_control')

        # publishers and messages
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.buttons_publisher = self.create_publisher(
            Button,
            'buttons',
            10)
        self.subscription = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.wireless_controller_callback,
            10)

        self.speed = 1.0  # Max lin speed, m/s
        self.ang_speed = -1.0  # Max ang speed, rad/s (sign convention switch)
        self.last_msg_time = self.get_clock().now()
        self.timeout_duration = Duration(seconds=0.5)  # Timeout duration
        self.get_logger().info(
            "Wireless controller control started. Use joystick for motion."
        )
        # Check timeout
        self.timer = self.create_timer(0.1, self.check_timeout)

    def wireless_controller_callback(self, msg):
        # publish twist message 
        self.twist = Twist()
        self.twist.linear.x = msg.ly * self.speed
        self.twist.linear.y = msg.lx * -self.speed
        self.twist.angular.z = msg.rx * self.ang_speed
        self.vel_publisher.publish(self.twist)

        # publish button message
        self.button = Button()
        self.button.up = True if msg.keys == 4096 else False
        self.button.down = True if msg.keys == 16384 else False
        self.button.start = True if msg.keys == 4 else False
        self.button.select = True if msg.keys == 8 else False
        self.button.a = True if msg.keys == 256 else False
        self.button.b = True if msg.keys == 512 else False
        self.buttons_publisher.publish(self.button)

        # update last message time
        self.last_msg_time = self.get_clock().now()

    def check_timeout(self):
        if self.get_clock().now() - self.last_msg_time > self.timeout_duration:
            # Publish zero velocities if timeout occurs
            zero_twist = Twist()
            self.vel_publisher.publish(zero_twist)

            # Publish zero button states if timeout occurs
            zero_button_msg = Button()
            for field_name, field_type in zero_button_msg.get_fields_and_field_types().items():
                if field_type == 'boolean':
                    setattr(zero_button_msg, field_name, False)
            self.buttons_publisher.publish(zero_button_msg)


def main(args=None):
    rclpy.init()
    node = WirelessControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
