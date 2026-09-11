#!/usr/bin/env python3
"""Startup helper for preparing the Go2 for low-level control."""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, SportModeState


LIE_DOWN_MODE = 5

# Some firmware/mode combinations never report LIE_DOWN_MODE on
# /sportmodestate (observed: mode stays 0 standing and lying). Lie-down is
# therefore also confirmed from body height and from joint positions.
# Standing body_height is ~0.32 m; lying is ~0.075 m.
DEFAULT_LIE_DOWN_BODY_HEIGHT_MAX = 0.15

# Mirrors SitPos / position_tolerance in rl_deploy/include/rl_deploy/constants.hpp
# Order: FR(hip,thigh,calf), FL, RR, RL
SIT_POS = (
    -0.046, 1.262, -2.784,
    0.048, 1.257, -2.794,
    -0.341, 1.278, -2.810,
    0.317, 1.265, -2.788,
)
DEFAULT_SIT_JOINT_TOLERANCE_RAD = 0.3


def max_sit_joint_error(joints) -> float:
    """Largest |q - SIT_POS| over the 12 leg joints; inf when joints unknown."""
    if joints is None or len(joints) < len(SIT_POS):
        return float('inf')
    return max(abs(q - ref) for q, ref in zip(joints, SIT_POS))


def lie_down_reason(
    state,
    joints,
    *,
    body_height_max: float = DEFAULT_LIE_DOWN_BODY_HEIGHT_MAX,
    sit_joint_tolerance_rad: float = DEFAULT_SIT_JOINT_TOLERANCE_RAD,
) -> Optional[str]:
    """
    Return which signal says the robot is lying down, or None.

    ``state`` is the latest SportModeState (or None); ``joints`` the latest
    12 joint positions from LowState in Unitree order (or None).
    """
    if state is not None:
        if state.mode == LIE_DOWN_MODE:
            return 'mode'
        if 0.0 < state.body_height <= body_height_max:
            return 'body_height'
    if max_sit_joint_error(joints) <= sit_joint_tolerance_rad:
        return 'joints'
    return None


class SportModeStateWatcher(Node):
    """
    Track sport-mode state and low state, and confirm the robot is lying down.

    Lie-down is confirmed when any of these holds on consecutive samples:
      * sport-mode ``mode == LIE_DOWN_MODE``
      * sport-mode ``0 < body_height <= body_height_max``
      * all 12 joints within ``sit_joint_tolerance_rad`` of ``SIT_POS``
    """

    def __init__(
        self,
        topic_name: str,
        *,
        lowstate_topic: str = '/lowstate',
        body_height_max: float = DEFAULT_LIE_DOWN_BODY_HEIGHT_MAX,
        sit_joint_tolerance_rad: float = DEFAULT_SIT_JOINT_TOLERANCE_RAD,
    ) -> None:
        super().__init__('go2_motion_prep')
        self._topic_name = topic_name
        self._body_height_max = float(body_height_max)
        self._sit_joint_tolerance_rad = float(sit_joint_tolerance_rad)
        self._last_state: Optional[SportModeState] = None
        self._last_state_time_monotonic = 0.0
        self._last_joints: Optional[list] = None
        self._consecutive_lie_down_samples = 0
        self._last_lie_down_reason: Optional[str] = None
        self._has_logged_first_state = False

        self.create_subscription(
            SportModeState,
            topic_name,
            self._sport_mode_state_callback,
            10,
        )
        self.create_subscription(
            LowState,
            lowstate_topic,
            self._lowstate_callback,
            10,
        )

    @property
    def last_state(self) -> Optional[SportModeState]:
        return self._last_state

    @property
    def last_joints(self) -> Optional[list]:
        return self._last_joints

    @property
    def lie_down_reason(self) -> Optional[str]:
        return self._last_lie_down_reason

    def max_sit_joint_error(self) -> float:
        return max_sit_joint_error(self._last_joints)

    def _update_lie_down_confirmation(self) -> None:
        reason = lie_down_reason(
            self._last_state,
            self._last_joints,
            body_height_max=self._body_height_max,
            sit_joint_tolerance_rad=self._sit_joint_tolerance_rad,
        )
        if reason is None:
            self._consecutive_lie_down_samples = 0
        else:
            self._consecutive_lie_down_samples += 1
            self._last_lie_down_reason = reason

    def _lowstate_callback(self, msg: LowState) -> None:
        self._last_joints = [msg.motor_state[i].q for i in range(12)]
        self._update_lie_down_confirmation()

    def state_age_sec(self) -> float:
        if self._last_state is None:
            return float('inf')
        return time.monotonic() - self._last_state_time_monotonic

    def has_lie_down_confirmation(self, min_samples: int) -> bool:
        return (
            self._last_state is not None
            and self._consecutive_lie_down_samples >= min_samples
        )

    def _sport_mode_state_callback(self, msg: SportModeState) -> None:
        self._last_state = msg
        self._last_state_time_monotonic = time.monotonic()
        self._update_lie_down_confirmation()

        if not self._has_logged_first_state:
            self._has_logged_first_state = True
            self.get_logger().info(
                'Received first sport-mode state on %s: mode=%d body_height=%.3f'
                % (self._topic_name, msg.mode, msg.body_height)
            )


def _spin_until(
    node: Node,
    predicate,
    timeout_sec: float,
    *,
    spin_step_sec: float = 0.1,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_once(node, timeout_sec=min(spin_step_sec, remaining))
    return predicate()


def _find_sdk_binary(binary_name: str, workspace_root: Optional[str] = None) -> Path:
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


def _terminate_process(process: subprocess.Popen[str], timeout_sec: float = 2.0) -> str:
    if process.poll() is None:
        process.terminate()
        try:
            stdout, _ = process.communicate(timeout=timeout_sec)
            return stdout or ''
        except subprocess.TimeoutExpired:
            process.kill()

    stdout, _ = process.communicate()
    return stdout or ''


def _wait_for_lie_down_confirmation(
    watcher: SportModeStateWatcher,
    process: subprocess.Popen[str],
    *,
    timeout_sec: float,
    confirmation_samples: int,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if watcher.has_lie_down_confirmation(confirmation_samples):
            return True
        if process.poll() is not None:
            return watcher.has_lie_down_confirmation(confirmation_samples)
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_once(watcher, timeout_sec=min(0.1, remaining))
    return watcher.has_lie_down_confirmation(confirmation_samples)


def _run_shutoff_motion(
    *,
    logger,
    shutoff_motion_path: Path,
    network_interface: str,
    shutoff_timeout_sec: float,
) -> None:
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
        print('[go2_motion_prep] go2_shutoff_motion output:')
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


def prepare_robot_for_low_level_control(
    *,
    network_interface: str,
    sport_state_topic: str = '/sportmodestate',
    workspace_root: Optional[str] = None,
    state_timeout_sec: float = 5.0,
    lie_down_timeout_sec: float = 15.0,
    lie_down_confirmation_samples: int = 3,
    lie_down_settle_sec: float = 1.0,
    shutoff_timeout_sec: float = 20.0,
    allow_missing_sport_state: bool = False,
    lowstate_topic: str = '/lowstate',
    lie_down_body_height_max: float = DEFAULT_LIE_DOWN_BODY_HEIGHT_MAX,
    sit_joint_tolerance_rad: float = DEFAULT_SIT_JOINT_TOLERANCE_RAD,
) -> None:
    """Drive the robot into lie-down, then deactivate sport-mode control."""
    did_init_rclpy = False
    if not rclpy.ok():
        rclpy.init(args=None)
        did_init_rclpy = True

    watcher = SportModeStateWatcher(
        sport_state_topic,
        lowstate_topic=lowstate_topic,
        body_height_max=lie_down_body_height_max,
        sit_joint_tolerance_rad=sit_joint_tolerance_rad,
    )
    try:
        watcher.get_logger().info(
            'Preparing robot for low-level control on %s'
            % network_interface
        )

        sport_client_path = _find_sdk_binary('go2_sport_client', workspace_root)
        shutoff_motion_path = _find_sdk_binary('go2_shutoff_motion', workspace_root)

        watcher.get_logger().info(
            'Using SDK binaries: %s and %s'
            % (sport_client_path, shutoff_motion_path)
        )

        if not _spin_until(
            watcher,
            lambda: watcher.last_state is not None,
            timeout_sec=state_timeout_sec,
        ):
            if allow_missing_sport_state:
                watcher.get_logger().warn(
                    'Timed out waiting for sport-mode state on '
                    f'{sport_state_topic}. This can happen when sport mode is '
                    'already deactivated; skipping lie-down confirmation and '
                    'verifying motion shutdown directly.'
                )
                _run_shutoff_motion(
                    logger=watcher.get_logger(),
                    shutoff_motion_path=shutoff_motion_path,
                    network_interface=network_interface,
                    shutoff_timeout_sec=shutoff_timeout_sec,
                )
                watcher.get_logger().info(
                    'High-level motion control is disabled; continuing without '
                    'sport-mode lie-down confirmation.'
                )
                return

            raise RuntimeError(
                'Timed out waiting for sport-mode state on '
                f'{sport_state_topic}. Make sure DDS/ROS communication is up.'
            )

        # Give the joint/body-height checks a moment in case the robot is
        # already down; otherwise fall through to requesting StandDown.
        _spin_until(
            watcher,
            lambda: watcher.has_lie_down_confirmation(
                lie_down_confirmation_samples
            ),
            timeout_sec=1.0,
        )

        if watcher.has_lie_down_confirmation(lie_down_confirmation_samples):
            last_state = watcher.last_state
            watcher.get_logger().info(
                'Robot already lying down (via %s): mode=%d body_height=%.3f '
                'max|q-SitPos|=%.3f'
                % (
                    watcher.lie_down_reason,
                    last_state.mode,
                    last_state.body_height,
                    watcher.max_sit_joint_error(),
                )
            )
        else:
            watcher.get_logger().info(
                'Requesting lie-down via %s %s'
                % (sport_client_path, network_interface)
            )
            sport_client_process = subprocess.Popen(
                [str(sport_client_path), network_interface],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )

            try:
                if not _wait_for_lie_down_confirmation(
                    watcher,
                    sport_client_process,
                    timeout_sec=lie_down_timeout_sec,
                    confirmation_samples=lie_down_confirmation_samples,
                ):
                    sport_client_output = _terminate_process(sport_client_process)
                    last_state = watcher.last_state
                    raise RuntimeError(
                        'Timed out waiting for lie-down confirmation after '
                        'requesting StandDown(). Last sport state: mode=%d '
                        'body_height=%.3f (max %.3f); max|q-SitPos|=%.3f '
                        '(tolerance %.3f).\ngo2_sport_client output:\n%s'
                        % (
                            last_state.mode,
                            last_state.body_height,
                            lie_down_body_height_max,
                            watcher.max_sit_joint_error(),
                            sit_joint_tolerance_rad,
                            sport_client_output,
                        )
                    )
            finally:
                sport_client_output = _terminate_process(sport_client_process)

            last_state = watcher.last_state
            watcher.get_logger().info(
                'Confirmed lie-down (via %s): mode=%d body_height=%.3f '
                'max|q-SitPos|=%.3f'
                % (
                    watcher.lie_down_reason,
                    last_state.mode,
                    last_state.body_height,
                    watcher.max_sit_joint_error(),
                )
            )
            if sport_client_output.strip():
                print('[go2_motion_prep] go2_sport_client output:')
                print(sport_client_output.rstrip())

        if lie_down_settle_sec > 0.0:
            watcher.get_logger().info(
                'Waiting %.1f seconds for the robot to settle on the ground'
                % lie_down_settle_sec
            )
            time.sleep(lie_down_settle_sec)

        _run_shutoff_motion(
            logger=watcher.get_logger(),
            shutoff_motion_path=shutoff_motion_path,
            network_interface=network_interface,
            shutoff_timeout_sec=shutoff_timeout_sec,
        )

        watcher.get_logger().info(
            'Robot is lying down and high-level motion control is disabled.'
        )
    finally:
        watcher.destroy_node()
        if did_init_rclpy:
            rclpy.shutdown()
