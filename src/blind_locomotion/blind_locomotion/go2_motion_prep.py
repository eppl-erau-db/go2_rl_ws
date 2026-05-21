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
from unitree_go.msg import SportModeState


LIE_DOWN_MODE = 5


class SportModeStateWatcher(Node):
    """Tracks the latest sport-mode state and confirms lie-down mode."""

    def __init__(self, topic_name: str) -> None:
        super().__init__('go2_motion_prep')
        self._topic_name = topic_name
        self._last_state: Optional[SportModeState] = None
        self._last_state_time_monotonic = 0.0
        self._consecutive_lie_down_samples = 0
        self._has_logged_first_state = False

        self.create_subscription(
            SportModeState,
            topic_name,
            self._sport_mode_state_callback,
            10,
        )

    @property
    def last_state(self) -> Optional[SportModeState]:
        return self._last_state

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
        if msg.mode == LIE_DOWN_MODE:
            self._consecutive_lie_down_samples += 1
        else:
            self._consecutive_lie_down_samples = 0

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
) -> None:
    """Drive the robot into lie-down, then deactivate sport-mode control."""

    did_init_rclpy = False
    if not rclpy.ok():
        rclpy.init(args=None)
        did_init_rclpy = True

    watcher = SportModeStateWatcher(sport_state_topic)
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

        if watcher.last_state.mode == LIE_DOWN_MODE:
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
                'Robot already reports lie-down mode=%d body_height=%.3f'
                % (last_state.mode, last_state.body_height)
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
                    raise RuntimeError(
                        'Timed out waiting for lie-down confirmation from '
                        f'{sport_state_topic} after requesting StandDown().\n'
                        f'go2_sport_client output:\n{sport_client_output}'
                    )
            finally:
                sport_client_output = _terminate_process(sport_client_process)

            last_state = watcher.last_state
            watcher.get_logger().info(
                'Confirmed lie-down via %s: mode=%d body_height=%.3f'
                % (sport_state_topic, last_state.mode, last_state.body_height)
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
