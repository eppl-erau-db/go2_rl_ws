#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Twist
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
POSE_X_RANGE = (0.20, 0.50)
POSE_Y_RANGE = (-0.30, -0.15)
POSE_Z_RANGE = (0.0, 0.30)
AXIS_POSE_X = 'left_y'
AXIS_POSE_Y = 'right_x'
AXIS_POSE_Z = 'right_y'
INVERT_POSE_X = False
INVERT_POSE_Y = True
INVERT_POSE_Z = False
TIMEOUT_SEC = 0.5


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def stick_to_range(stick_value, lo, hi):
    center = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo)
    return clamp(center + stick_value * half, lo, hi)


def stick_to_range_center_min(stick_value, lo, hi):
    # Neutral stick (0.0) maps to the lower bound; positive stick opens up the range.
    return lo + clamp(stick_value, 0.0, 1.0) * (hi - lo)


def stick_to_range_center_max(stick_value, lo, hi):
    # Neutral stick (0.0) maps to the upper bound; negative stick opens down the range.
    return hi + clamp(stick_value, -1.0, 0.0) * (hi - lo)


def ordered_range(lo, hi):
    if lo <= hi:
        return (lo, hi)
    return (hi, lo)


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

        self.declare_parameter('publish_pose_command', False)
        self.declare_parameter('pose_command_topic', 'pose_command')
        self.declare_parameter('pose_x_min', POSE_X_RANGE[0])
        self.declare_parameter('pose_x_max', POSE_X_RANGE[1])
        self.declare_parameter('pose_y_min', POSE_Y_RANGE[0])
        self.declare_parameter('pose_y_max', POSE_Y_RANGE[1])
        self.declare_parameter('pose_z_min', POSE_Z_RANGE[0])
        self.declare_parameter('pose_z_max', POSE_Z_RANGE[1])

        self.publish_pose_command = bool(self.get_parameter('publish_pose_command').value)
        self.pose_command_topic = str(self.get_parameter('pose_command_topic').value)
        self.pose_x_range = ordered_range(
            float(self.get_parameter('pose_x_min').value),
            float(self.get_parameter('pose_x_max').value),
        )
        self.pose_y_range = ordered_range(
            float(self.get_parameter('pose_y_min').value),
            float(self.get_parameter('pose_y_max').value),
        )
        self.pose_z_range = ordered_range(
            float(self.get_parameter('pose_z_min').value),
            float(self.get_parameter('pose_z_max').value),
        )
        self.axis_pose_x = AXIS_POSE_X
        self.axis_pose_y = AXIS_POSE_Y
        self.axis_pose_z = AXIS_POSE_Z
        self.invert_pose_x = INVERT_POSE_X
        self.invert_pose_y = INVERT_POSE_Y
        self.invert_pose_z = INVERT_POSE_Z

        self.timeout_duration = Duration(seconds=TIMEOUT_SEC)

        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.buttons_publisher = self.create_publisher(Button, 'buttons', 10)
        self.pose_publisher = None
        if self.publish_pose_command:
            self.pose_publisher = self.create_publisher(Pose, self.pose_command_topic, 10)

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
        if self.publish_pose_command:
            self.get_logger().info(
                'Pose command enabled: topic=%s x=(%.3f, %.3f) y=(%.3f, %.3f) z=(%.3f, %.3f)'
                % (
                    self.pose_command_topic,
                    self.pose_x_range[0], self.pose_x_range[1],
                    self.pose_y_range[0], self.pose_y_range[1],
                    self.pose_z_range[0], self.pose_z_range[1],
                )
            )

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

    def _neutral_pose_msg(self):
        msg = Pose()
        msg.position.x = self.pose_x_range[0]
        msg.position.y = self.pose_y_range[1]
        msg.position.z = self.pose_z_range[0]
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        return msg

    def _publish_pose_command(self, msg):
        if not self.publish_pose_command or self.pose_publisher is None:
            return

        pose_x_stick = self._axis_value(msg, self.axis_pose_x, self.invert_pose_x)
        pose_y_stick = self._axis_value(msg, self.axis_pose_y, self.invert_pose_y)
        pose_z_stick = self._axis_value(msg, self.axis_pose_z, self.invert_pose_z)

        pose_msg = Pose()
        pose_msg.position.x = stick_to_range_center_min(pose_x_stick, *self.pose_x_range)
        pose_msg.position.y = stick_to_range_center_max(pose_y_stick, *self.pose_y_range)
        pose_msg.position.z = stick_to_range_center_min(pose_z_stick, *self.pose_z_range)
        pose_msg.orientation.w = 1.0
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        self.pose_publisher.publish(pose_msg)

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

        self._publish_pose_command(msg)

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

        if self.publish_pose_command and self.pose_publisher is not None:
            self.pose_publisher.publish(self._neutral_pose_msg())


def main(args=None):
    rclpy.init(args=args)
    node = WirelessControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
