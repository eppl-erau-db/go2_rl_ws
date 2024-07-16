#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_go.msg import WirelessController
from rclpy.duration import Duration

class WirelessControl(Node):
    def __init__(self):
        super().__init__('wireless_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.wireless_controller_callback,
            10)
        self.twist = Twist()
        self.speed = 1.0  # Adjust speed as needed (up to 1 m/s)
        self.turn = 1.0  # Adjust turn rate as needed (up to 1 rad/s)
        self.last_msg_time = self.get_clock().now()
        self.timeout_duration = Duration(seconds=0.5)  # Set timeout duration (e.g., 0.5 seconds)
        self.get_logger().info("Wireless controller teleop started. Use joystick for motion.")
        self.timer = self.create_timer(0.1, self.check_timeout)  # Check timeout every 0.1 seconds

    def wireless_controller_callback(self, msg):
        self.twist = Twist()
        self.twist.linear.x = msg.lx * self.speed
        self.twist.linear.y = msg.ly * self.speed
        self.twist.angular.z = msg.rx * self.turn
        # Publish the Twist message
        self.publisher.publish(self.twist)
        # Update the last message time
        self.last_msg_time = self.get_clock().now()

    def check_timeout(self):
        if self.get_clock().now() - self.last_msg_time > self.timeout_duration:
            # Publish zero velocities if timeout occurs
            zero_twist = Twist()
            self.publisher.publish(zero_twist)

def main(args=None):
    rclpy.init(args=args)
    node = WirelessControl()
    rclpy.spin(node)
    # Clean up when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
