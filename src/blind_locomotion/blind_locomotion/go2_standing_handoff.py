#!/usr/bin/env python3
# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
"""Guarded standing handoff from Unitree ai_sport to low-level control."""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from unitree_api.msg import Request
from unitree_go.msg import SportModeState


LIE_DOWN_MODE = 5
ROBOT_SPORT_API_ID_STOPMOVE = 1003


def make_sport_request(api_id):
    req = Request()
    req.header.identity.id = time.monotonic_ns()
    req.header.identity.api_id = int(api_id)
    return req


def _find_sdk_binary(binary_name, workspace_root=None):
    candidate_roots = []
    if workspace_root:
        candidate_roots.append(Path(workspace_root).expanduser().resolve())

    env_root = os.environ.get('GO2_RL_WS_ROOT')
    if env_root:
        candidate_roots.append(Path(env_root).expanduser().resolve())

    module_path = Path(__file__).resolve()
    candidate_roots.extend(module_path.parents)

    seen = set()
    for root in candidate_roots:
        if root in seen:
            continue
        seen.add(root)
        candidate = root / 'sdk' / 'unitree_sdk2' / 'build' / 'bin' / binary_name
        if candidate.is_file():
            return candidate

    raise FileNotFoundError(
        f'Could not find sdk/unitree_sdk2/build/bin/{binary_name}. '
        'Set GO2_RL_WS_ROOT if the workspace root cannot be inferred.'
    )


def _run_shutoff_motion(
    *,
    logger,
    shutoff_motion_path,
    network_interface,
    shutoff_timeout_sec,
):
    logger.info(
        'Running %s %s and sending Enter automatically'
        % (shutoff_motion_path, network_interface)
    )
    shutoff_result = subprocess.run(
        [str(shutoff_motion_path), network_interface],
        input='\n',
        text=True,
        capture_output=True,
        timeout=shutoff_timeout_sec,
        check=False,
    )

    combined_output = (shutoff_result.stdout or '') + (shutoff_result.stderr or '')
    if combined_output.strip():
        print('[go2_standing_handoff] go2_shutoff_motion output:')
        print(combined_output.rstrip())

    verified_deactivation = (
        '[OK] Verified motion service deactivated.' in combined_output
        or '[OK] Motion service already deactivated' in combined_output
    )
    completed_deactivation = '[DONE] Motion deactivated.' in combined_output

    if shutoff_result.returncode != 0 or not (
        verified_deactivation and completed_deactivation
    ):
        raise RuntimeError(
            'go2_shutoff_motion did not confirm high-level motion shutdown.\n'
            f'Return code: {shutoff_result.returncode}\n'
            f'Output:\n{combined_output}'
        )


def _finite_float(value, default=float('nan')):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(value):
        return default
    return value


def _velocity_norm(state):
    velocity = getattr(state, 'velocity', [])
    components = []
    for index in range(3):
        try:
            components.append(_finite_float(velocity[index], default=0.0))
        except (IndexError, TypeError):
            components.append(0.0)
    return math.sqrt(sum(component * component for component in components))


def is_stable_upright_state(
    state,
    *,
    state_age_sec,
    max_state_age_sec=0.25,
    min_body_height=0.18,
    max_linear_velocity=0.05,
    max_yaw_speed=0.10,
):
    """Return true when a sport-mode state is fresh, upright, and nearly still."""
    if state is None:
        return False

    state_age_sec = _finite_float(state_age_sec)
    if state_age_sec < 0.0 or state_age_sec > max_state_age_sec:
        return False

    try:
        mode = int(getattr(state, 'mode'))
    except (TypeError, ValueError):
        return False
    if mode == LIE_DOWN_MODE:
        return False

    body_height = _finite_float(getattr(state, 'body_height', None))
    if body_height < min_body_height:
        return False

    if _velocity_norm(state) > max_linear_velocity:
        return False

    yaw_speed = abs(_finite_float(getattr(state, 'yaw_speed', None), default=0.0))
    if yaw_speed > max_yaw_speed:
        return False

    return True


class StandingStabilityTracker:
    """Tracks consecutive stable upright samples."""

    def __init__(
        self,
        *,
        required_samples,
        max_state_age_sec=0.25,
        min_body_height=0.18,
        max_linear_velocity=0.05,
        max_yaw_speed=0.10,
    ):
        self.required_samples = max(1, int(required_samples))
        self.max_state_age_sec = float(max_state_age_sec)
        self.min_body_height = float(min_body_height)
        self.max_linear_velocity = float(max_linear_velocity)
        self.max_yaw_speed = float(max_yaw_speed)
        self.stable_samples = 0

    def update(self, state, *, state_age_sec):
        if is_stable_upright_state(
            state,
            state_age_sec=state_age_sec,
            max_state_age_sec=self.max_state_age_sec,
            min_body_height=self.min_body_height,
            max_linear_velocity=self.max_linear_velocity,
            max_yaw_speed=self.max_yaw_speed,
        ):
            self.stable_samples += 1
        else:
            self.stable_samples = 0
        return self.confirmed(state, state_age_sec=state_age_sec)

    def confirmed(self, state, *, state_age_sec):
        return self.stable_samples >= self.required_samples and is_stable_upright_state(
            state,
            state_age_sec=state_age_sec,
            max_state_age_sec=self.max_state_age_sec,
            min_body_height=self.min_body_height,
            max_linear_velocity=self.max_linear_velocity,
            max_yaw_speed=self.max_yaw_speed,
        )


def _transient_local_qos(depth=1):
    qos = QoSProfile(depth=depth)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class StandingHandoffNode(Node):
    """Stops high-level motion, verifies standing still, then enables low-level."""

    def __init__(self):
        super().__init__('go2_standing_handoff')

        self.declare_parameter('network_interface', 'enp1s0')
        self.declare_parameter('sport_state_topic', '/sportmodestate')
        self.declare_parameter('workspace_root', '')
        self.declare_parameter('state_timeout_sec', 5.0)
        self.declare_parameter('stable_timeout_sec', 8.0)
        self.declare_parameter('stable_required_samples', 5)
        self.declare_parameter('max_state_age_sec', 0.25)
        self.declare_parameter('min_body_height', 0.18)
        self.declare_parameter('max_linear_velocity', 0.05)
        self.declare_parameter('max_yaw_speed', 0.10)
        self.declare_parameter('stop_publish_frequency', 20.0)
        self.declare_parameter('enable_publish_duration_sec', 2.0)
        self.declare_parameter('shutoff_timeout_sec', 20.0)

        self.network_interface = self.get_parameter(
            'network_interface'
        ).get_parameter_value().string_value
        self.sport_state_topic = self.get_parameter(
            'sport_state_topic'
        ).get_parameter_value().string_value
        self.workspace_root = self.get_parameter(
            'workspace_root'
        ).get_parameter_value().string_value
        self.state_timeout_sec = float(self.get_parameter('state_timeout_sec').value)
        self.stable_timeout_sec = float(
            self.get_parameter('stable_timeout_sec').value
        )
        self.stop_publish_period_sec = 1.0 / max(
            float(self.get_parameter('stop_publish_frequency').value),
            1e-6,
        )
        self.enable_publish_duration_sec = float(
            self.get_parameter('enable_publish_duration_sec').value
        )
        self.shutoff_timeout_sec = float(
            self.get_parameter('shutoff_timeout_sec').value
        )
        self.tracker = StandingStabilityTracker(
            required_samples=int(self.get_parameter('stable_required_samples').value),
            max_state_age_sec=float(self.get_parameter('max_state_age_sec').value),
            min_body_height=float(self.get_parameter('min_body_height').value),
            max_linear_velocity=float(
                self.get_parameter('max_linear_velocity').value
            ),
            max_yaw_speed=float(self.get_parameter('max_yaw_speed').value),
        )

        self.last_state = None
        self.last_state_time_monotonic = 0.0
        self.last_stop_publish_time = 0.0
        self.has_logged_first_state = False

        self.sport_request_publisher = self.create_publisher(
            Request,
            '/api/sport/request',
            10,
        )
        self.enable_publisher = self.create_publisher(
            Bool,
            '/low_level_handoff/enable',
            _transient_local_qos(),
        )
        self.sport_state_subscription = self.create_subscription(
            SportModeState,
            self.sport_state_topic,
            self._sport_state_callback,
            10,
        )

    def _sport_state_callback(self, msg):
        self.last_state = msg
        self.last_state_time_monotonic = time.monotonic()
        self.tracker.update(msg, state_age_sec=0.0)

        if not self.has_logged_first_state:
            self.has_logged_first_state = True
            self.get_logger().info(
                'Received first sport-mode state on %s: mode=%d body_height=%.3f'
                % (self.sport_state_topic, msg.mode, msg.body_height)
            )

    def state_age_sec(self):
        if self.last_state is None:
            return float('inf')
        return time.monotonic() - self.last_state_time_monotonic

    def has_stable_confirmation(self):
        return self.tracker.confirmed(
            self.last_state,
            state_age_sec=self.state_age_sec(),
        )

    def publish_stop_move(self, *, force=False):
        now = time.monotonic()
        if (
            not force
            and now - self.last_stop_publish_time < self.stop_publish_period_sec
        ):
            return
        self.sport_request_publisher.publish(
            make_sport_request(ROBOT_SPORT_API_ID_STOPMOVE)
        )
        self.last_stop_publish_time = now

    def wait_for_stable_upright(self):
        start_time = time.monotonic()
        state_deadline = start_time + self.state_timeout_sec
        stable_deadline = start_time + self.stable_timeout_sec

        self.publish_stop_move(force=True)
        while time.monotonic() < stable_deadline:
            self.publish_stop_move()
            if self.has_stable_confirmation():
                return

            if self.last_state is None and time.monotonic() >= state_deadline:
                raise RuntimeError(
                    'Timed out waiting for sport-mode state on '
                    f'{self.sport_state_topic}'
                )

            rclpy.spin_once(self, timeout_sec=0.02)

        last_mode = getattr(self.last_state, 'mode', None)
        last_body_height = getattr(self.last_state, 'body_height', None)
        raise RuntimeError(
            'Timed out waiting for a stable upright sport-mode state. '
            f'Last mode={last_mode} body_height={last_body_height} '
            f'stable_samples={self.tracker.stable_samples}'
        )

    def publish_enable_signal(self):
        msg = Bool()
        msg.data = True
        deadline = time.monotonic() + max(0.1, self.enable_publish_duration_sec)
        while time.monotonic() < deadline:
            self.enable_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def execute_handoff(self):
        self.get_logger().warn(
            'Experimental standing handoff enabled: stopping ai_sport motion, '
            'verifying the robot is upright/stationary, then releasing high-level '
            'motion before low-level control is enabled.'
        )
        self.wait_for_stable_upright()
        self.publish_stop_move(force=True)

        shutoff_motion_path = _find_sdk_binary(
            'go2_shutoff_motion',
            self.workspace_root or None,
        )
        _run_shutoff_motion(
            logger=self.get_logger(),
            shutoff_motion_path=shutoff_motion_path,
            network_interface=self.network_interface,
            shutoff_timeout_sec=self.shutoff_timeout_sec,
        )

        self.get_logger().info(
            'High-level motion is released; enabling low-level standing takeover.'
        )
        self.publish_enable_signal()


def main(args=None):
    rclpy.init(args=args)
    node = StandingHandoffNode()
    try:
        node.execute_handoff()
    except KeyboardInterrupt:
        node.get_logger().warn('Standing handoff interrupted; low-level not enabled.')
        return 130
    except Exception as exc:  # noqa: BLE001 - helper must fail closed with context.
        node.get_logger().error(f'Standing handoff failed: {exc}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
