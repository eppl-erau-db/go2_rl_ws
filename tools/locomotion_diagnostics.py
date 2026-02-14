#!/usr/bin/env python3
"""Locomotion policy diagnostic recorder.

Subscribes to all relevant topics, prints live frequency stats, saves
timestamped data to .npz, and runs automated diagnostic checks after
recording completes.

Usage:
    # Launch system with debug topics enabled first:
    #   ros2 launch go2_launch go2_walk.launch.py \
    #       policy_debug_publish_obs_topic:=true \
    #       policy_debug_publish_action_topic:=true

    python3 tools/locomotion_diagnostics.py --duration 15
    python3 tools/locomotion_diagnostics.py --duration 10 --output diag_run1.npz
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import defaultdict
from datetime import datetime

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32, Float32MultiArray
    from unitree_go.msg import LowState
except ImportError as exc:
    raise SystemExit(
        "Failed to import ROS 2 libraries. Source your environment first, e.g.\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source ~/ros2_ws/install/setup.bash\n"
        f"Import error: {exc}"
    )

# Joint names in Unitree SDK order (matches /lowstate motor_state and actions topic).
UNITREE_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

# Isaac Lab joint order used by raw actions and observation joint slices.
ISAAC_JOINT_NAMES = [
    "FL_hip", "FR_hip", "RL_hip", "RR_hip",
    "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh",
    "FL_calf", "FR_calf", "RL_calf", "RR_calf",
]

# Default joint positions (hip=0.0, thigh=0.8, calf=-1.5).
DEFAULT_POS = np.array([
    0.0, 0.8, -1.5,  # FR
    0.0, 0.8, -1.5,  # FL
    0.0, 0.8, -1.5,  # RR
    0.0, 0.8, -1.5,  # RL
], dtype=np.float32)


class DiagnosticRecorder(Node):
    def __init__(self, duration: float):
        super().__init__("locomotion_diagnostics")
        self.duration = duration
        self.start_time = None
        self.recording = True

        # Per-topic data storage: {topic_name: [(timestamp, data), ...]}
        self.data = defaultdict(list)
        # Per-topic message counts for frequency tracking.
        self.msg_counts = defaultdict(int)
        self.last_freq_print_time = None

        # Subscriptions.
        self.create_subscription(
            LowState, "/lowstate", self._cb_lowstate, 10
        )
        self.create_subscription(
            Float32MultiArray, "actions", self._cb_actions, 10
        )
        self.create_subscription(
            Twist, "cmd_vel", self._cb_cmd_vel, 10
        )
        self.create_subscription(
            Float32MultiArray, "debug/rl_obs", self._cb_obs, 10
        )
        self.create_subscription(
            Float32MultiArray, "debug/rl_raw_action", self._cb_raw_action, 10
        )
        self.create_subscription(
            Float32, "/base_height", self._cb_base_height, 10
        )

        # 50 Hz timer for bookkeeping (frequency print, stop check).
        self.create_timer(0.02, self._tick)

        self.get_logger().info(
            f"Recording for {self.duration:.1f}s. "
            "Waiting for first message..."
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _rel_time(self) -> float:
        if self.start_time is None:
            return 0.0
        return self._now_sec() - self.start_time

    def _ensure_started(self):
        if self.start_time is None:
            self.start_time = self._now_sec()
            self.last_freq_print_time = self.start_time
            self.get_logger().info("First message received — recording started.")

    def _cb_lowstate(self, msg: LowState):
        if not self.recording:
            return
        self._ensure_started()
        t = self._rel_time()
        # Store joint positions and velocities in Unitree order.
        joint_pos = np.array(
            [msg.motor_state[i].q for i in range(12)], dtype=np.float32
        )
        joint_vel = np.array(
            [msg.motor_state[i].dq for i in range(12)], dtype=np.float32
        )
        quat_wxyz = np.array(
            msg.imu_state.quaternion[0:4], dtype=np.float32
        )
        gyro = np.array(
            msg.imu_state.gyroscope[0:3], dtype=np.float32
        )
        accel = np.array(
            msg.imu_state.accelerometer[0:3], dtype=np.float32
        )
        foot_force = np.array(
            msg.foot_force[0:4], dtype=np.float32
        )
        self.data["lowstate"].append(
            (t, joint_pos, joint_vel, quat_wxyz, gyro, accel, foot_force)
        )
        self.msg_counts["lowstate"] += 1

    def _cb_actions(self, msg: Float32MultiArray):
        if not self.recording:
            return
        self._ensure_started()
        t = self._rel_time()
        data = np.array(msg.data, dtype=np.float32)
        self.data["actions"].append((t, data))
        self.msg_counts["actions"] += 1

    def _cb_cmd_vel(self, msg: Twist):
        if not self.recording:
            return
        self._ensure_started()
        t = self._rel_time()
        data = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )
        self.data["cmd_vel"].append((t, data))
        self.msg_counts["cmd_vel"] += 1

    def _cb_obs(self, msg: Float32MultiArray):
        if not self.recording:
            return
        self._ensure_started()
        t = self._rel_time()
        data = np.array(msg.data, dtype=np.float32)
        self.data["obs"].append((t, data))
        self.msg_counts["obs"] += 1

    def _cb_raw_action(self, msg: Float32MultiArray):
        if not self.recording:
            return
        self._ensure_started()
        t = self._rel_time()
        data = np.array(msg.data, dtype=np.float32)
        self.data["raw_action"].append((t, data))
        self.msg_counts["raw_action"] += 1

    def _cb_base_height(self, msg: Float32):
        if not self.recording:
            return
        self._ensure_started()
        t = self._rel_time()
        self.data["base_height"].append((t, float(msg.data)))
        self.msg_counts["base_height"] += 1

    def _tick(self):
        if self.start_time is None:
            return
        elapsed = self._rel_time()

        # Print frequency stats at ~1 Hz.
        if elapsed - (self.last_freq_print_time - self.start_time) >= 1.0:
            dt = elapsed - (self.last_freq_print_time - self.start_time)
            parts = []
            for name in [
                "lowstate", "actions", "cmd_vel", "obs", "raw_action", "base_height"
            ]:
                count = self.msg_counts[name]
                hz = count / dt if dt > 0 else 0.0
                parts.append(f"{name}: {hz:.1f} Hz ({count})")
                self.msg_counts[name] = 0
            self.last_freq_print_time = self._now_sec()
            remaining = max(0.0, self.duration - elapsed)
            print(
                f"[{elapsed:.1f}s / {self.duration:.1f}s] "
                f"remaining={remaining:.1f}s | "
                + " | ".join(parts)
            )

        # Check if recording is done.
        if elapsed >= self.duration:
            self.recording = False
            self.get_logger().info("Recording complete.")
            raise KeyboardInterrupt  # Clean exit from spin.


def _build_arrays(data: dict) -> dict:
    """Convert collected data lists into numpy arrays for saving."""
    arrays = {}

    # lowstate: (t, joint_pos[12], joint_vel[12], quat[4], gyro[3], accel[3], foot_force[4])
    if data["lowstate"]:
        ls = data["lowstate"]
        arrays["lowstate_t"] = np.array([s[0] for s in ls], dtype=np.float64)
        arrays["lowstate_joint_pos"] = np.array([s[1] for s in ls], dtype=np.float32)
        arrays["lowstate_joint_vel"] = np.array([s[2] for s in ls], dtype=np.float32)
        arrays["lowstate_quat_wxyz"] = np.array([s[3] for s in ls], dtype=np.float32)
        arrays["lowstate_gyro"] = np.array([s[4] for s in ls], dtype=np.float32)
        arrays["lowstate_accel"] = np.array([s[5] for s in ls], dtype=np.float32)
        arrays["lowstate_foot_force"] = np.array([s[6] for s in ls], dtype=np.float32)

    # actions: (t, data[12]) in Unitree order.
    if data["actions"]:
        act = data["actions"]
        arrays["actions_t"] = np.array([s[0] for s in act], dtype=np.float64)
        arrays["actions_data"] = np.array([s[1] for s in act], dtype=np.float32)

    # cmd_vel: (t, [vx, vy, wz])
    if data["cmd_vel"]:
        cv = data["cmd_vel"]
        arrays["cmd_vel_t"] = np.array([s[0] for s in cv], dtype=np.float64)
        arrays["cmd_vel_data"] = np.array([s[1] for s in cv], dtype=np.float32)

    # obs: (t, data[49])
    if data["obs"]:
        ob = data["obs"]
        arrays["obs_t"] = np.array([s[0] for s in ob], dtype=np.float64)
        arrays["obs_data"] = np.array([s[1] for s in ob], dtype=np.float32)

    # raw_action: (t, data[12]) in Isaac Lab order.
    if data["raw_action"]:
        ra = data["raw_action"]
        arrays["raw_action_t"] = np.array([s[0] for s in ra], dtype=np.float64)
        arrays["raw_action_data"] = np.array([s[1] for s in ra], dtype=np.float32)

    # base_height: (t, scalar)
    if data["base_height"]:
        bh = data["base_height"]
        arrays["base_height_t"] = np.array([s[0] for s in bh], dtype=np.float64)
        arrays["base_height_data"] = np.array([s[1] for s in bh], dtype=np.float32)

    return arrays


def _run_diagnostics(arrays: dict):
    """Print automated diagnostic checks to terminal."""
    print("\n" + "=" * 72)
    print("AUTOMATED DIAGNOSTICS")
    print("=" * 72)

    # 1. Gravity Convention Check
    print("\n--- 1. Gravity Convention Check ---")
    if "obs_data" in arrays and arrays["obs_data"].shape[0] > 0:
        grav = arrays["obs_data"][:, 7:10]  # projected_gravity
        gz_mean = float(np.mean(grav[:, 2]))
        gz_std = float(np.std(grav[:, 2]))
        gx_mean = float(np.mean(grav[:, 0]))
        gy_mean = float(np.mean(grav[:, 1]))
        print(f"  Projected gravity mean: gx={gx_mean:.4f}, gy={gy_mean:.4f}, gz={gz_mean:.4f}")
        print(f"  gz std: {gz_std:.4f}")
        if gz_mean < -0.8:
            print("  OK: gz ~ -1.0 matches INVERSE convention (expected for this policy).")
        elif gz_mean > 0.8:
            print("  WARNING: gz ~ +1.0 suggests DIRECT convention — mismatch with policy!")
        else:
            print(f"  WARNING: gz={gz_mean:.3f} is ambiguous. Robot may not be upright.")
    else:
        print("  SKIPPED: No observation data (debug/rl_obs topic not received).")

    # 2. Action Scale Check
    print("\n--- 2. Action Scale Check ---")
    if "raw_action_data" in arrays and arrays["raw_action_data"].shape[0] > 0:
        raw = arrays["raw_action_data"]
        max_abs = float(np.max(np.abs(raw)))
        mean_abs = float(np.mean(np.abs(raw)))
        std_val = float(np.std(raw))
        print(f"  Raw action stats: max|a|={max_abs:.4f}, mean|a|={mean_abs:.4f}, std={std_val:.4f}")
        if max_abs < 0.05:
            print("  CRITICAL: Policy is NOT generating meaningful movement!")
            print("           Likely cause: observation mismatch or wrong model.")
        elif max_abs > 3.0:
            print("  WARNING: Policy is railing (outputs > 3.0). Check observation inputs.")
        elif max_abs < 0.3:
            print("  NOTICE: Raw actions are small. Policy may be near equilibrium or uncertain.")
        else:
            print(f"  OK: Raw actions in reasonable range (max|a|={max_abs:.3f}).")
    else:
        print("  SKIPPED: No raw action data (debug/rl_raw_action topic not received).")

    # 3. cmd_vel Pipeline Check
    print("\n--- 3. cmd_vel Pipeline Check ---")
    if "cmd_vel_data" in arrays and arrays["cmd_vel_data"].shape[0] > 0:
        cv = arrays["cmd_vel_data"]
        cv_norms = np.linalg.norm(cv, axis=1)
        nonzero_cmd = int(np.count_nonzero(cv_norms > 0.05))
        print(f"  cmd_vel samples: {len(cv)}, nonzero (|cmd|>0.05): {nonzero_cmd}")

        if "obs_data" in arrays and arrays["obs_data"].shape[0] > 0:
            obs_cmd = arrays["obs_data"][:, 10:13]
            obs_cmd_norms = np.linalg.norm(obs_cmd, axis=1)
            nonzero_obs_cmd = int(np.count_nonzero(obs_cmd_norms > 0.05))
            print(f"  obs velocity_commands nonzero: {nonzero_obs_cmd}")
            if nonzero_cmd > 10 and nonzero_obs_cmd == 0:
                print("  CRITICAL: cmd_vel has commands but obs shows zeros!")
                print("           The velocity command pipeline is broken.")
            elif nonzero_cmd > 0 and nonzero_obs_cmd > 0:
                print("  OK: Commands are reaching the policy observation vector.")
            elif nonzero_cmd == 0:
                print("  NOTICE: No nonzero cmd_vel during recording. Send joystick commands to test.")
        else:
            print("  (Cannot cross-check — no obs data.)")
    else:
        print("  SKIPPED: No cmd_vel data received.")

    # 4. Joint Symmetry Check
    print("\n--- 4. Joint Symmetry Check ---")
    if "lowstate_joint_pos" in arrays and arrays["lowstate_joint_pos"].shape[0] > 0:
        jp = arrays["lowstate_joint_pos"]
        # Unitree order: FR(0,1,2), FL(3,4,5), RR(6,7,8), RL(9,10,11)
        fr_mean = np.mean(jp[:, 0:3], axis=0)
        fl_mean = np.mean(jp[:, 3:6], axis=0)
        rr_mean = np.mean(jp[:, 6:9], axis=0)
        rl_mean = np.mean(jp[:, 9:12], axis=0)
        print(f"  Mean joint pos (hip, thigh, calf):")
        print(f"    FR: [{fr_mean[0]:+.3f}, {fr_mean[1]:+.3f}, {fr_mean[2]:+.3f}]")
        print(f"    FL: [{fl_mean[0]:+.3f}, {fl_mean[1]:+.3f}, {fl_mean[2]:+.3f}]")
        print(f"    RR: [{rr_mean[0]:+.3f}, {rr_mean[1]:+.3f}, {rr_mean[2]:+.3f}]")
        print(f"    RL: [{rl_mean[0]:+.3f}, {rl_mean[1]:+.3f}, {rl_mean[2]:+.3f}]")

        # Check FL vs FR asymmetry (thigh/calf should be similar).
        fl_fr_diff = np.abs(fl_mean - fr_mean)
        rl_rr_diff = np.abs(rl_mean - rr_mean)
        # Hip signs may differ slightly, focus on thigh/calf.
        max_front_diff = float(np.max(fl_fr_diff[1:]))
        max_rear_diff = float(np.max(rl_rr_diff[1:]))
        if max_front_diff > 0.3 or max_rear_diff > 0.3:
            print(f"  WARNING: Large L/R asymmetry (front thigh/calf diff={max_front_diff:.3f}, "
                  f"rear={max_rear_diff:.3f}).")
            print("           Possible joint ordering bug.")
        else:
            print(f"  OK: Left/Right symmetry looks reasonable "
                  f"(front diff={max_front_diff:.3f}, rear={max_rear_diff:.3f}).")
    else:
        print("  SKIPPED: No lowstate data received.")

    # 5. Frequency Validation
    print("\n--- 5. Frequency Validation ---")
    expected_hz = {
        "actions": 25.0,
        "lowstate": 500.0,
        "cmd_vel": 50.0,
        "obs": 25.0,
        "raw_action": 25.0,
        "base_height": 50.0,
    }
    topic_time_keys = {
        "actions": "actions_t",
        "lowstate": "lowstate_t",
        "cmd_vel": "cmd_vel_t",
        "obs": "obs_t",
        "raw_action": "raw_action_t",
        "base_height": "base_height_t",
    }
    for name, t_key in topic_time_keys.items():
        if t_key not in arrays or len(arrays[t_key]) < 2:
            print(f"  {name}: NO DATA")
            continue
        timestamps = arrays[t_key]
        intervals = np.diff(timestamps)
        actual_hz = 1.0 / np.mean(intervals) if np.mean(intervals) > 0 else 0.0
        jitter_ms = float(np.std(intervals)) * 1000.0
        exp = expected_hz.get(name, 0.0)
        within_20pct = float(np.mean(
            np.abs(intervals - (1.0 / exp)) < 0.2 * (1.0 / exp)
        )) * 100.0 if exp > 0 else 0.0
        status = "OK" if abs(actual_hz - exp) / max(exp, 1.0) < 0.3 else "WARNING"
        print(f"  {name}: {actual_hz:.1f} Hz (expected ~{exp:.0f} Hz) "
              f"jitter={jitter_ms:.1f}ms within_20%={within_20pct:.0f}% [{status}]")

    # 6. Quick Tracking Error Summary
    print("\n--- 6. Tracking Error Summary ---")
    if ("actions_data" in arrays and arrays["actions_data"].shape[0] > 0
            and "lowstate_joint_pos" in arrays and arrays["lowstate_joint_pos"].shape[0] > 0):
        # Interpolate action targets to lowstate timestamps.
        act_t = arrays["actions_t"]
        act_d = arrays["actions_data"]
        ls_t = arrays["lowstate_t"]
        ls_jp = arrays["lowstate_joint_pos"]
        # Only compare in overlapping time range.
        t_start = max(act_t[0], ls_t[0])
        t_end = min(act_t[-1], ls_t[-1])
        ls_mask = (ls_t >= t_start) & (ls_t <= t_end)
        if np.count_nonzero(ls_mask) > 10:
            ls_t_masked = ls_t[ls_mask]
            ls_jp_masked = ls_jp[ls_mask]
            # Interpolate each action joint to lowstate timestamps.
            interp_targets = np.zeros_like(ls_jp_masked)
            for j in range(12):
                interp_targets[:, j] = np.interp(ls_t_masked, act_t, act_d[:, j])
            errors = np.abs(interp_targets - ls_jp_masked)
            mean_err = np.mean(errors, axis=0)
            max_err = np.max(errors, axis=0)
            print(f"  Per-joint mean |error| (target vs actual):")
            for j, name in enumerate(UNITREE_JOINT_NAMES):
                print(f"    {name:12s}: mean={mean_err[j]:.4f} rad, max={max_err[j]:.4f} rad")
            overall_mean = float(np.mean(mean_err))
            if overall_mean < 0.05:
                print(f"  OK: Tracking is tight (overall mean error={overall_mean:.4f} rad).")
                print("      If robot isn't walking, the problem is in policy output, not tracking.")
            elif overall_mean < 0.15:
                print(f"  NOTICE: Moderate tracking error ({overall_mean:.4f} rad).")
            else:
                print(f"  WARNING: Large tracking error ({overall_mean:.4f} rad).")
                print("           Check PD gains or action frequency.")
        else:
            print("  SKIPPED: Insufficient overlapping data.")
    else:
        print("  SKIPPED: Need both actions and lowstate data.")

    print("\n" + "=" * 72)


def main():
    parser = argparse.ArgumentParser(
        description="Record locomotion policy diagnostics from ROS 2 topics."
    )
    parser.add_argument(
        "--duration", type=float, default=10.0,
        help="Recording duration in seconds (default: 10)",
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Output .npz file path (default: locomotion_diag_TIMESTAMP.npz)",
    )
    args = parser.parse_args()

    if args.output is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"locomotion_diag_{ts}.npz"

    rclpy.init()
    node = DiagnosticRecorder(duration=args.duration)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info("Building arrays and saving...")

    arrays = _build_arrays(node.data)

    if not arrays:
        print("ERROR: No data was recorded. Check that topics are being published.",
              file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    np.savez_compressed(args.output, **arrays)
    total_samples = sum(len(v) for v in node.data.values())
    print(f"\nSaved {total_samples} total samples to {args.output}")
    print(f"Arrays: {sorted(arrays.keys())}")

    _run_diagnostics(arrays)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
