#!/usr/bin/env python3
# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
"""Bridge muxed cmd_vel commands to Unitree Go2 ai_sport requests."""

import json
import math
import os
import threading
import time

import rclpy
from blind_locomotion.msg import Button
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from unitree_api.msg import Request, Response


ROBOT_STATE_API_ID_SERVICE_SWITCH = 1001

ROBOT_SPORT_API_ID_DAMP = 1001
ROBOT_SPORT_API_ID_STOPMOVE = 1003
ROBOT_SPORT_API_ID_STANDDOWN = 1005
ROBOT_SPORT_API_ID_MOVE = 1008

AI_SPORT_SERVICE_NAME = 'ai_sport'
SPORT_MODE_SERVICE_NAME = 'sport_mode'


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def finite_or_zero(value):
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return value


def apply_deadband(value, deadband):
    if abs(value) <= deadband:
        return 0.0
    return value


def make_request(api_id, parameter=None):
    req = Request()
    req.header.identity.id = time.monotonic_ns()
    req.header.identity.api_id = int(api_id)
    if parameter is not None:
        req.parameter = json.dumps(parameter)
    return req


def make_robot_state_service_switch_request(service_name, switch):
    return make_request(
        ROBOT_STATE_API_ID_SERVICE_SWITCH,
        {
            'name': str(service_name),
            'switch': int(switch),
        },
    )


def make_move_request(vx, vy, vyaw):
    return make_request(
        ROBOT_SPORT_API_ID_MOVE,
        {
            'x': float(vx),
            'y': float(vy),
            'z': float(vyaw),
        },
    )


def make_sport_request(api_id):
    return make_request(api_id)


def sanitize_twist(cmd, max_linear_x, max_linear_y, max_angular_z, deadband):
    vx = clamp(finite_or_zero(cmd.linear.x), -max_linear_x, max_linear_x)
    vy = clamp(finite_or_zero(cmd.linear.y), -max_linear_y, max_linear_y)
    vyaw = clamp(finite_or_zero(cmd.angular.z), -max_angular_z, max_angular_z)
    return (
        apply_deadband(vx, deadband),
        apply_deadband(vy, deadband),
        apply_deadband(vyaw, deadband),
    )


def is_zero_velocity(vx, vy, vyaw):
    return vx == 0.0 and vy == 0.0 and vyaw == 0.0


class AiSportCmdVelNode(Node):
    def __init__(self):
        super().__init__('ai_sport_cmd_vel')

        self.declare_parameter('activate_ai_sport', True)
        self.declare_parameter('disable_sport_mode_before_ai_sport', True)
        self.declare_parameter('require_ai_sport_confirmation', True)
        self.declare_parameter('require_start_button', True)
        self.declare_parameter('exit_on_select', False)
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('cmd_deadband', 0.02)
        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_linear_y', 0.3)
        self.declare_parameter('max_angular_z', 0.8)

        self.activate_ai_sport = bool(
            self.get_parameter('activate_ai_sport').value
        )
        self.disable_sport_mode_before_ai_sport = bool(
            self.get_parameter('disable_sport_mode_before_ai_sport').value
        )
        self.require_ai_sport_confirmation = bool(
            self.get_parameter('require_ai_sport_confirmation').value
        )
        self.require_start_button = bool(
            self.get_parameter('require_start_button').value
        )
        self.exit_on_select = bool(self.get_parameter('exit_on_select').value)
        publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.cmd_deadband = float(self.get_parameter('cmd_deadband').value)
        self.max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self.max_linear_y = abs(float(self.get_parameter('max_linear_y').value))
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))

        self.ai_sport_request_period_sec = 2.0
        self.stop_request_period_sec = 0.25

        now = self.get_clock().now()
        self.latest_cmd_vel = Twist()
        self.cmd_vel_received = False
        self.last_cmd_vel_time = now
        self.last_stop_request_time = now
        self.last_ai_sport_request_time = now
        self.ai_sport_confirmed = (
            not self.activate_ai_sport or not self.require_ai_sport_confirmation
        )
        self.armed = not self.require_start_button
        self.motion_active = False
        self.low_level_handoff_requested = False

        self.last_start_button = False
        self.last_select_button = False
        self.last_a_button = False
        self.last_b_button = False
        self.last_down_button = False
        self.last_emergency_sit_button = False

        self.sport_request_publisher = self.create_publisher(
            Request, '/api/sport/request', 10
        )
        self.robot_state_request_publisher = self.create_publisher(
            Request, '/api/robot_state/request', 10
        )
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Button, 'buttons', self.buttons_callback, 10)
        self.create_subscription(
            Response,
            '/api/robot_state/response',
            self.robot_state_response_callback,
            10,
        )

        if self.activate_ai_sport:
            self.publish_ai_sport_request(force=True)

        timer_period = 1.0 / max(publish_frequency, 1e-6)
        self.timer = self.create_timer(timer_period, self.tick)

        self.get_logger().info(
            'ai_sport cmd_vel bridge started: '
            f'activate_ai_sport={self.activate_ai_sport} '
            f'require_ai_sport_confirmation={self.require_ai_sport_confirmation} '
            f'require_start_button={self.require_start_button} '
            f'frequency={publish_frequency:.1f}Hz '
            f'timeout={self.cmd_timeout_sec:.2f}s '
            f'deadband={self.cmd_deadband:.3f}'
        )
        self.get_logger().info(
            'Velocity limits: '
            f'x={self.max_linear_x:.2f} y={self.max_linear_y:.2f} '
            f'yaw={self.max_angular_z:.2f}'
        )
        if self.require_start_button:
            self.get_logger().info(
                'Press START on the wireless controller to arm motion; '
                'press START again to StopMove and disarm'
            )
        if self.exit_on_select:
            self.get_logger().info(
                'Press SELECT to stop ai_sport and hand off to low-level control'
            )

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_received = True

    def buttons_callback(self, msg):
        start_button = bool(getattr(msg, 'start', False))
        select_button = bool(getattr(msg, 'select', False))
        a_button = bool(getattr(msg, 'a', False))
        b_button = bool(getattr(msg, 'b', False))
        down_button = bool(getattr(msg, 'down', False))
        emergency_sit_button = bool(getattr(msg, 'emergency_sit', False))

        b_rising = b_button and not self.last_b_button
        down_rising = down_button and not self.last_down_button
        emergency_sit_rising = (
            emergency_sit_button and not self.last_emergency_sit_button
        )
        a_rising = a_button and not self.last_a_button
        start_rising = start_button and not self.last_start_button
        select_rising = select_button and not self.last_select_button

        if select_rising and self.exit_on_select:
            self.request_low_level_handoff()
        elif b_rising:
            self.disarm_with_request(
                ROBOT_SPORT_API_ID_DAMP,
                'B pressed: publishing Damp and disarming',
            )
        elif down_rising or emergency_sit_rising:
            self.disarm_with_request(
                ROBOT_SPORT_API_ID_STANDDOWN,
                'DOWN/emergency_sit pressed: publishing StandDown and disarming',
            )
        elif a_rising:
            self.disarm_with_request(
                ROBOT_SPORT_API_ID_STOPMOVE,
                'A pressed: publishing StopMove and disarming',
            )
        elif start_rising:
            if self.armed:
                self.disarm_with_request(
                    ROBOT_SPORT_API_ID_STOPMOVE,
                    'START pressed while armed: publishing StopMove and disarming',
                )
            else:
                self.armed = True
                self.get_logger().info('START pressed: ai_sport velocity bridge armed')
                if self.activate_ai_sport:
                    self.publish_ai_sport_request(force=True)

        self.last_start_button = start_button
        self.last_select_button = select_button
        self.last_a_button = a_button
        self.last_b_button = b_button
        self.last_down_button = down_button
        self.last_emergency_sit_button = emergency_sit_button

    def robot_state_response_callback(self, msg):
        if msg.header.identity.api_id != ROBOT_STATE_API_ID_SERVICE_SWITCH:
            return

        try:
            response = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warn(
                f'Ignoring malformed robot_state response: {msg.data!r}',
                throttle_duration_sec=2.0,
            )
            return

        service_name = str(response.get('name', ''))
        try:
            service_status = int(response.get('status'))
        except (TypeError, ValueError):
            self.get_logger().warn(
                f'Robot_state response missing integer status: {response}',
                throttle_duration_sec=2.0,
            )
            return

        if service_name == SPORT_MODE_SERVICE_NAME:
            self.get_logger().info(
                f'Robot_state reports {SPORT_MODE_SERVICE_NAME} status={service_status}'
            )
            return

        if service_name != AI_SPORT_SERVICE_NAME:
            return

        if service_status == 1:
            if not self.ai_sport_confirmed:
                self.get_logger().info('Robot_state confirmed ai_sport is active')
            self.ai_sport_confirmed = True
            return

        self.ai_sport_confirmed = False
        self.get_logger().warn(
            f'Robot_state reports ai_sport status={service_status}; '
            'not forwarding velocity until status=1'
        )

    def disarm_with_request(self, api_id, message):
        self.armed = False
        self.motion_active = False
        self.sport_request_publisher.publish(make_sport_request(api_id))
        self.last_stop_request_time = self.get_clock().now()
        self.get_logger().warn(message)

    def request_low_level_handoff(self):
        if self.low_level_handoff_requested:
            return

        self.low_level_handoff_requested = True
        self.disarm_with_request(
            ROBOT_SPORT_API_ID_STOPMOVE,
            'SELECT pressed: publishing StopMove and exiting for low-level handoff',
        )

        def force_exit_if_executor_does_not_unwind():
            time.sleep(0.5)
            os._exit(0)

        threading.Thread(
            target=force_exit_if_executor_does_not_unwind,
            daemon=True,
        ).start()
        time.sleep(0.05)
        raise SystemExit(0)

    def elapsed_sec(self, now, since_time):
        return (now - since_time).nanoseconds * 1e-9

    def publish_ai_sport_request(self, force=False):
        if not self.activate_ai_sport:
            return
        if self.ai_sport_confirmed and not force:
            return

        now = self.get_clock().now()
        if (
            not force
            and self.elapsed_sec(now, self.last_ai_sport_request_time)
            < self.ai_sport_request_period_sec
        ):
            return

        if self.disable_sport_mode_before_ai_sport:
            self.robot_state_request_publisher.publish(
                make_robot_state_service_switch_request(SPORT_MODE_SERVICE_NAME, 0)
            )
        self.robot_state_request_publisher.publish(
            make_robot_state_service_switch_request(AI_SPORT_SERVICE_NAME, 1)
        )
        self.last_ai_sport_request_time = now
        if force:
            if self.disable_sport_mode_before_ai_sport:
                self.get_logger().info(
                    'Requested sport_mode off, then Unitree ai_sport on'
                )
            else:
                self.get_logger().info('Requested Unitree ai_sport service')

    def publish_stop_move(self, reason, force=False):
        now = self.get_clock().now()
        if (
            not force
            and self.elapsed_sec(now, self.last_stop_request_time)
            < self.stop_request_period_sec
        ):
            self.motion_active = False
            return

        self.sport_request_publisher.publish(
            make_sport_request(ROBOT_SPORT_API_ID_STOPMOVE)
        )
        self.last_stop_request_time = now
        self.motion_active = False
        self.get_logger().debug(f'Publishing StopMove: {reason}')

    def tick(self):
        self.publish_ai_sport_request()

        if not self.ai_sport_confirmed:
            if self.motion_active:
                self.publish_stop_move('waiting for ai_sport confirmation', force=True)
            self.get_logger().warn(
                'Waiting for robot_state confirmation that ai_sport is active; '
                'not forwarding cmd_vel',
                throttle_duration_sec=2.0,
            )
            return

        if not self.armed:
            if self.motion_active:
                self.publish_stop_move('bridge disarmed', force=True)
            return

        if not self.cmd_vel_received:
            if self.motion_active:
                self.publish_stop_move('no cmd_vel received yet', force=True)
            return

        now = self.get_clock().now()
        cmd_age = self.elapsed_sec(now, self.last_cmd_vel_time)
        if cmd_age > self.cmd_timeout_sec:
            self.publish_stop_move(
                f'cmd_vel stale ({cmd_age:.3f}s > {self.cmd_timeout_sec:.3f}s)'
            )
            return

        vx, vy, vyaw = sanitize_twist(
            self.latest_cmd_vel,
            self.max_linear_x,
            self.max_linear_y,
            self.max_angular_z,
            self.cmd_deadband,
        )

        if is_zero_velocity(vx, vy, vyaw):
            self.publish_stop_move('zero cmd_vel after deadband')
            return

        self.sport_request_publisher.publish(make_move_request(vx, vy, vyaw))
        self.motion_active = True


def main(args=None):
    rclpy.init(args=args)
    node = AiSportCmdVelNode()
    try:
        rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
