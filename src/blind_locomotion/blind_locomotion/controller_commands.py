#!/usr/bin/env python3
import math

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


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def stick_to_range(stick_value, lo, hi):
    center = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo)
    return clamp(center + stick_value * half, lo, hi)


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class WirelessControl(Node):
    def __init__(self):
        super().__init__('wireless_control')

        # Command ranges used in training.
        self.declare_parameter('lin_vel_x_min', -1.0)
        self.declare_parameter('lin_vel_x_max', 1.0)
        self.declare_parameter('lin_vel_y_min', -1.0)
        self.declare_parameter('lin_vel_y_max', 1.0)
        self.declare_parameter('ang_vel_z_min', -1.0)
        self.declare_parameter('ang_vel_z_max', 1.0)
        # Heading range is retained as metadata for parity with training config.
        self.declare_parameter('heading_min', -math.pi)
        self.declare_parameter('heading_max', math.pi)

        # Axis mapping and sign convention.
        self.declare_parameter('axis_lin_x', 'left_y')
        self.declare_parameter('axis_lin_y', 'left_x')
        self.declare_parameter('axis_ang_z', 'right_x')
        self.declare_parameter('invert_lin_x', False)
        self.declare_parameter('invert_lin_y', True)
        self.declare_parameter('invert_ang_z', True)

        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('debug_enabled', True)
        self.declare_parameter('debug_rate_hz', 5.0)
        self.declare_parameter('debug_only_nonzero_cmd', False)

        self.lin_x_range = (
            float(self.get_parameter('lin_vel_x_min').value),
            float(self.get_parameter('lin_vel_x_max').value),
        )
        self.lin_y_range = (
            float(self.get_parameter('lin_vel_y_min').value),
            float(self.get_parameter('lin_vel_y_max').value),
        )
        self.ang_z_range = (
            float(self.get_parameter('ang_vel_z_min').value),
            float(self.get_parameter('ang_vel_z_max').value),
        )
        self.heading_range = (
            float(self.get_parameter('heading_min').value),
            float(self.get_parameter('heading_max').value),
        )

        self.axis_lin_x = str(self.get_parameter('axis_lin_x').value)
        self.axis_lin_y = str(self.get_parameter('axis_lin_y').value)
        self.axis_ang_z = str(self.get_parameter('axis_ang_z').value)

        self.invert_lin_x = as_bool(self.get_parameter('invert_lin_x').value)
        self.invert_lin_y = as_bool(self.get_parameter('invert_lin_y').value)
        self.invert_ang_z = as_bool(self.get_parameter('invert_ang_z').value)

        timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.timeout_duration = Duration(seconds=timeout_sec)
        self.debug_enabled = as_bool(self.get_parameter('debug_enabled').value)
        self.debug_rate_hz = max(0.1, float(self.get_parameter('debug_rate_hz').value))
        self.debug_only_nonzero_cmd = as_bool(
            self.get_parameter('debug_only_nonzero_cmd').value
        )
        self.debug_interval_sec = 1.0 / self.debug_rate_hz

        self._validate_axis_name('axis_lin_x', self.axis_lin_x)
        self._validate_axis_name('axis_lin_y', self.axis_lin_y)
        self._validate_axis_name('axis_ang_z', self.axis_ang_z)

        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.buttons_publisher = self.create_publisher(Button, 'buttons', 10)

        self.subscription = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.wireless_controller_callback,
            10,
        )

        self.last_msg_time = self.get_clock().now()
        self.last_debug_time = self.last_msg_time
        self.last_axis_warn_time = self.last_msg_time
        self.in_timeout = False
        self.timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info(
            'Wireless controller started. '
            f'lin_x={self.lin_x_range}, lin_y={self.lin_y_range}, ang_z={self.ang_z_range}, '
            f'heading(meta)={self.heading_range}, timeout={timeout_sec:.2f}s'
        )
        self.get_logger().info(
            'Axis mapping: '
            f'lin_x={self.axis_lin_x} (invert={self.invert_lin_x}), '
            f'lin_y={self.axis_lin_y} (invert={self.invert_lin_y}), '
            f'ang_z={self.axis_ang_z} (invert={self.invert_ang_z})'
        )
        self.get_logger().info(
            f'Debug: enabled={self.debug_enabled} rate={self.debug_rate_hz:.1f}Hz '
            f'only_nonzero_cmd={self.debug_only_nonzero_cmd}'
        )

    def _validate_axis_name(self, field_name, axis_name):
        if axis_name not in STICK_FIELDS:
            raise ValueError(
                f'Invalid {field_name}="{axis_name}". '
                f'Valid options: {list(STICK_FIELDS.keys())}'
            )

    def _axis_value(self, msg, axis_name, invert):
        raw_value = float(getattr(msg, STICK_FIELDS[axis_name]))
        if invert:
            raw_value *= -1.0
        return clamp(raw_value, -1.0, 1.0)

    def _elapsed_sec(self, now, since_time):
        return (now - since_time).nanoseconds * 1e-9

    def _should_debug_log(self, now, twist, keys):
        if not self.debug_enabled:
            return False
        if self._elapsed_sec(now, self.last_debug_time) < self.debug_interval_sec:
            return False

        cmd_norm = math.sqrt(
            float(twist.linear.x) ** 2
            + float(twist.linear.y) ** 2
            + float(twist.angular.z) ** 2
        )
        if self.debug_only_nonzero_cmd and cmd_norm < 1e-3 and int(keys) == 0:
            return False

        self.last_debug_time = now
        return True

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
        ) and self._elapsed_sec(now, self.last_axis_warn_time) >= 0.5:
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

        if self._should_debug_log(now, twist, msg.keys):
            self.get_logger().info(
                '[Debug] '
                f'raw_axes=(lx={raw_lx:.3f} ly={raw_ly:.3f} rx={raw_rx:.3f} ry={raw_ry:.3f}) '
                f'mapped_sticks=(x={lin_x_stick:.3f} y={lin_y_stick:.3f} wz={ang_z_stick:.3f}) '
                f'cmd_vel=(x={float(twist.linear.x):.3f} y={float(twist.linear.y):.3f} '
                f'wz={float(twist.angular.z):.3f}) '
                f'keys={int(msg.keys)} '
                f'buttons=(up={bool(buttons_msg.up)} down={bool(buttons_msg.down)} '
                f'start={bool(buttons_msg.start)} select={bool(buttons_msg.select)} '
                f'a={bool(buttons_msg.a)} b={bool(buttons_msg.b)})'
            )

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
