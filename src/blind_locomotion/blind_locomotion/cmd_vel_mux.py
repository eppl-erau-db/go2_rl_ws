#!/usr/bin/env python3
"""Arbitrate between navigation and wireless cmd_vel sources."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from blind_locomotion.ros_node_shutdown import spin_until_shutdown
except ModuleNotFoundError:
    from ros_node_shutdown import spin_until_shutdown


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('nav_timeout_sec', 0.5)
        self.declare_parameter('wireless_timeout_sec', 0.5)
        self.declare_parameter('manual_override_deadband', 0.1)

        publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.nav_timeout_sec = float(self.get_parameter('nav_timeout_sec').value)
        self.wireless_timeout_sec = float(self.get_parameter('wireless_timeout_sec').value)
        self.manual_override_deadband = float(
            self.get_parameter('manual_override_deadband').value
        )

        now = self.get_clock().now()
        self.latest_nav_cmd = Twist()
        self.latest_wireless_cmd = Twist()
        self.nav_received = False
        self.wireless_received = False
        self.last_nav_time = now
        self.last_wireless_time = now
        self.last_source = 'idle'

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'nav_cmd_vel', self.nav_callback, 10)
        self.create_subscription(Twist, 'wireless_cmd_vel', self.wireless_callback, 10)
        self.timer = self.create_timer(1.0 / publish_frequency, self.publish_cmd_vel)

        self.get_logger().info(
            f'cmd_vel mux started: nav_timeout={self.nav_timeout_sec:.2f}s '
            f'wireless_timeout={self.wireless_timeout_sec:.2f}s '
            f'manual_override_deadband={self.manual_override_deadband:.2f}'
        )

    def nav_callback(self, msg):
        self.latest_nav_cmd = msg
        self.nav_received = True
        self.last_nav_time = self.get_clock().now()

    def wireless_callback(self, msg):
        self.latest_wireless_cmd = msg
        self.wireless_received = True
        self.last_wireless_time = self.get_clock().now()

    def _elapsed_sec(self, now, since_time):
        return (now - since_time).nanoseconds * 1e-9

    def _is_fresh(self, now, received, timestamp, timeout_sec):
        return received and self._elapsed_sec(now, timestamp) <= timeout_sec

    def _is_manual_override_active(self, cmd):
        return (
            abs(cmd.linear.x) > self.manual_override_deadband
            or abs(cmd.linear.y) > self.manual_override_deadband
            or abs(cmd.angular.z) > self.manual_override_deadband
        )

    def _copy_twist(self, cmd):
        twist = Twist()
        twist.linear.x = float(cmd.linear.x)
        twist.linear.y = float(cmd.linear.y)
        twist.linear.z = float(cmd.linear.z)
        twist.angular.x = float(cmd.angular.x)
        twist.angular.y = float(cmd.angular.y)
        twist.angular.z = float(cmd.angular.z)
        return twist

    def _selected_command(self, now):
        wireless_fresh = self._is_fresh(
            now,
            self.wireless_received,
            self.last_wireless_time,
            self.wireless_timeout_sec,
        )
        nav_fresh = self._is_fresh(
            now,
            self.nav_received,
            self.last_nav_time,
            self.nav_timeout_sec,
        )

        if wireless_fresh and self._is_manual_override_active(self.latest_wireless_cmd):
            return self._copy_twist(self.latest_wireless_cmd), 'wireless'

        if nav_fresh:
            return self._copy_twist(self.latest_nav_cmd), 'nav'

        return Twist(), 'idle'

    def publish_cmd_vel(self):
        now = self.get_clock().now()
        twist, source = self._selected_command(now)
        self.publisher.publish(twist)

        if source != self.last_source:
            self.get_logger().info(f'cmd_vel source -> {source}')
            self.last_source = source


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    spin_until_shutdown(node)


if __name__ == '__main__':
    main()
