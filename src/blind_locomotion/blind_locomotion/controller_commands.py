#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from unitree_go.msg import WirelessController
from blind_locomotion.msg import Button


STICK_FIELDS = {
    'left_x': 'lx',
    'left_y': 'ly',
    'right_x': 'rx',
    'right_y': 'ry',
}

# Fixed hardware/training constants.
LIN_X_RANGE = (-1.0, 1.0)
LIN_Y_RANGE = (-1.0, 1.0)
ANG_Z_RANGE = (-1.0, 1.0)
AXIS_LIN_X = 'left_y'
AXIS_LIN_Y = 'left_x'
AXIS_ANG_Z = 'right_x'
INVERT_LIN_X = False
INVERT_LIN_Y = True
INVERT_ANG_Z = True
TIMEOUT_SEC = 0.5


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def stick_to_range(stick_value, lo, hi):
    center = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo)
    return clamp(center + stick_value * half, lo, hi)


class WirelessControl(Node):
    def __init__(self):
        super().__init__('wireless_control')

        self.lin_x_range = LIN_X_RANGE
        self.lin_y_range = LIN_Y_RANGE
        self.ang_z_range = ANG_Z_RANGE

        self.axis_lin_x = AXIS_LIN_X
        self.axis_lin_y = AXIS_LIN_Y
        self.axis_ang_z = AXIS_ANG_Z

        self.invert_lin_x = INVERT_LIN_X
        self.invert_lin_y = INVERT_LIN_Y
        self.invert_ang_z = INVERT_ANG_Z

        self.timeout_duration = Duration(seconds=TIMEOUT_SEC)

        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.buttons_publisher = self.create_publisher(Button, 'buttons', 10)

        self.subscription = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.wireless_controller_callback,
            10,
        )

        self.last_msg_time = self.get_clock().now()
        self.last_axis_warn_time = self.last_msg_time
        self.in_timeout = False
        self.timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info('Wireless controller started')

    def _axis_value(self, msg, axis_name, invert):
        raw_value = float(getattr(msg, STICK_FIELDS[axis_name]))
        if invert:
            raw_value *= -1.0
        return clamp(raw_value, -1.0, 1.0)

    def _buttons_from_wireless(self, msg):
        button = Button()
        button.up = msg.keys == 4096
        button.down = msg.keys == 16384
        button.start = msg.keys == 4
        button.select = msg.keys == 8
        button.a = msg.keys == 256
        button.b = msg.keys == 512
        button.emergency_sit = False
        return button

    def wireless_controller_callback(self, msg):
        now = self.get_clock().now()
        if self.in_timeout:
            self.get_logger().info('Wireless controller recovered from timeout')
            self.in_timeout = False

        raw_lx = float(msg.lx)
        raw_ly = float(msg.ly)
        raw_rx = float(msg.rx)
        raw_ry = float(msg.ry)

        if (
            raw_lx < -1.0 or raw_lx > 1.0
            or raw_ly < -1.0 or raw_ly > 1.0
            or raw_rx < -1.0 or raw_rx > 1.0
            or raw_ry < -1.0 or raw_ry > 1.0
        ) and (now - self.last_axis_warn_time).nanoseconds * 1e-9 >= 0.5:
            self.last_axis_warn_time = now
            self.get_logger().warn(
                'Wireless axes out of expected [-1, 1] range before clamp: '
                f'lx={raw_lx:.3f} ly={raw_ly:.3f} rx={raw_rx:.3f} ry={raw_ry:.3f}'
            )

        lin_x_stick = self._axis_value(msg, self.axis_lin_x, self.invert_lin_x)
        lin_y_stick = self._axis_value(msg, self.axis_lin_y, self.invert_lin_y)
        ang_z_stick = self._axis_value(msg, self.axis_ang_z, self.invert_ang_z)

        twist = Twist()
        twist.linear.x = stick_to_range(lin_x_stick, *self.lin_x_range)
        twist.linear.y = stick_to_range(lin_y_stick, *self.lin_y_range)
        # Direct yaw-rate mode for locomotion policy commands.
        twist.angular.z = stick_to_range(ang_z_stick, *self.ang_z_range)
        self.vel_publisher.publish(twist)

        buttons_msg = self._buttons_from_wireless(msg)
        self.buttons_publisher.publish(buttons_msg)
        self.last_msg_time = now

    def check_timeout(self):
        now = self.get_clock().now()
        if now - self.last_msg_time <= self.timeout_duration:
            return

        if not self.in_timeout:
            self.get_logger().warn(
                'Wireless timeout entered: publishing zero cmd_vel and zero buttons'
            )
            self.in_timeout = True

        self.vel_publisher.publish(Twist())

        zero_button_msg = Button()
        for field_name, field_type in zero_button_msg.get_fields_and_field_types().items():
            if field_type == 'boolean':
                setattr(zero_button_msg, field_name, False)
        self.buttons_publisher.publish(zero_button_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WirelessControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
