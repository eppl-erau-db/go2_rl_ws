#!/usr/bin/env python3
"""Locomotion policy diagnostic plotter.

Loads a .npz file produced by locomotion_diagnostics.py and generates
diagnostic figures for identifying locomotion policy issues.

Usage:
    python3 tools/plot_locomotion_diagnostics.py locomotion_diag_20260213_143000.npz
    python3 tools/plot_locomotion_diagnostics.py recording.npz --save-dir ./plots
"""

from __future__ import annotations

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

# Joint names in Unitree SDK order (matches /lowstate motor_state and actions topic).
UNITREE_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

# Isaac Lab joint order used by raw actions and obs joint slices.
ISAAC_JOINT_NAMES = [
    "FL_hip", "FR_hip", "RL_hip", "RR_hip",
    "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh",
    "FL_calf", "FR_calf", "RL_calf", "RR_calf",
]

# Default joint positions per Unitree ordering.
DEFAULT_POS_UNITREE = np.array([
    0.0, 0.8, -1.5,  # FR
    0.0, 0.8, -1.5,  # FL
    0.0, 0.8, -1.5,  # RR
    0.0, 0.8, -1.5,  # RL
], dtype=np.float32)

# Leg grouping for 4x3 grid: (leg_name, unitree_indices: [hip, thigh, calf])
LEG_GROUPS = [
    ("FR", [0, 1, 2]),
    ("FL", [3, 4, 5]),
    ("RR", [6, 7, 8]),
    ("RL", [9, 10, 11]),
]

JOINT_TYPE_NAMES = ["hip", "thigh", "calf"]
JOINT_TYPE_DEFAULTS = [0.0, 0.8, -1.5]


def _has(d, *keys):
    return all(k in d and d[k].shape[0] > 0 for k in keys)


def plot_target_vs_actual(d, save_dir=None):
    """Plot 1: Target vs Actual Joint Positions (4 legs x 3 joints)."""
    if not _has(d, "actions_t", "actions_data", "lowstate_t", "lowstate_joint_pos"):
        print("Plot 1 (Target vs Actual): SKIPPED — missing actions or lowstate data.")
        return

    fig, axes = plt.subplots(4, 3, figsize=(16, 12), sharex=True)
    fig.suptitle("Plot 1: Target vs Actual Joint Positions", fontsize=14, fontweight="bold")

    act_t = d["actions_t"]
    act_d = d["actions_data"]
    ls_t = d["lowstate_t"]
    ls_jp = d["lowstate_joint_pos"]

    for row, (leg_name, indices) in enumerate(LEG_GROUPS):
        for col, (jtype, jidx) in enumerate(zip(JOINT_TYPE_NAMES, indices)):
            ax = axes[row, col]
            ax.plot(ls_t, ls_jp[:, jidx], "b-", linewidth=0.8, alpha=0.8, label="actual")
            ax.plot(act_t, act_d[:, jidx], "r--", linewidth=0.8, alpha=0.8, label="target")
            ax.axhline(
                y=DEFAULT_POS_UNITREE[jidx], color="gray", linestyle=":",
                linewidth=0.6, alpha=0.5, label="default"
            )
            ax.set_title(f"{leg_name} {jtype}", fontsize=10)
            ax.set_ylabel("rad")
            if row == 0 and col == 0:
                ax.legend(fontsize=7, loc="upper right")
            if row == 3:
                ax.set_xlabel("time (s)")

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "01_target_vs_actual.png"), dpi=150)
    return fig


def plot_raw_action_analysis(d, save_dir=None):
    """Plot 2: Raw Action Magnitude Analysis."""
    if not _has(d, "raw_action_t", "raw_action_data"):
        print("Plot 2 (Raw Action Analysis): SKIPPED — missing raw_action data.")
        return

    ra_t = d["raw_action_t"]
    ra_d = d["raw_action_data"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    fig.suptitle("Plot 2: Raw Action Magnitude Analysis", fontsize=14, fontweight="bold")

    # Left: time series of all 12 channels.
    for i in range(12):
        ax1.plot(ra_t, ra_d[:, i], linewidth=0.7, alpha=0.7, label=ISAAC_JOINT_NAMES[i])
    ax1.axhline(y=1.0, color="red", linestyle="--", linewidth=0.5, alpha=0.5)
    ax1.axhline(y=-1.0, color="red", linestyle="--", linewidth=0.5, alpha=0.5)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("raw action value")
    ax1.set_title("Raw Action Time Series")
    ax1.legend(fontsize=6, ncol=3, loc="upper right")

    # Right: histogram of all raw action values.
    all_vals = ra_d.flatten()
    ax2.hist(all_vals, bins=80, color="steelblue", alpha=0.7, edgecolor="none")
    ax2.axvline(x=-1.0, color="red", linestyle="--", linewidth=1.0, label="expected range [-1, 1]")
    ax2.axvline(x=1.0, color="red", linestyle="--", linewidth=1.0)
    ax2.set_xlabel("raw action value")
    ax2.set_ylabel("count")
    ax2.set_title("Raw Action Distribution")
    ax2.legend(fontsize=8)
    stats_text = (
        f"mean={np.mean(all_vals):.3f}\n"
        f"std={np.std(all_vals):.3f}\n"
        f"max|a|={np.max(np.abs(all_vals)):.3f}\n"
        f"|a|>1: {np.count_nonzero(np.abs(all_vals)>1.0)}/{len(all_vals)}"
    )
    ax2.text(
        0.98, 0.95, stats_text, transform=ax2.transAxes,
        fontsize=8, verticalalignment="top", horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "02_raw_action_analysis.png"), dpi=150)
    return fig


def plot_observation_breakdown(d, save_dir=None):
    """Plot 3: Observation Breakdown (8 subplots)."""
    if not _has(d, "obs_t", "obs_data"):
        print("Plot 3 (Observation Breakdown): SKIPPED — missing obs data.")
        return

    obs_t = d["obs_t"]
    obs = d["obs_data"]

    fig, axes = plt.subplots(4, 2, figsize=(16, 16))
    fig.suptitle("Plot 3: Observation Vector Breakdown", fontsize=14, fontweight="bold")

    # 1. base_lin_vel [0:3]
    ax = axes[0, 0]
    for i, label in enumerate(["vx", "vy", "vz"]):
        ax.plot(obs_t, obs[:, i], linewidth=0.8, label=label)
    ax.set_title("base_lin_vel [0:3]")
    ax.set_ylabel("m/s")
    ax.legend(fontsize=8)

    # 2. base_ang_vel [3:6]
    ax = axes[0, 1]
    for i, label in enumerate(["wx", "wy", "wz"]):
        ax.plot(obs_t, obs[:, 3 + i], linewidth=0.8, label=label)
    ax.set_title("base_ang_vel [3:6] (IMU gyro)")
    ax.set_ylabel("rad/s")
    ax.legend(fontsize=8)

    # 3. base_height [6]
    ax = axes[1, 0]
    ax.plot(obs_t, obs[:, 6], linewidth=0.8, color="green")
    ax.axhline(y=0.25, color="gray", linestyle=":", alpha=0.5, label="0.25m")
    ax.axhline(y=0.35, color="gray", linestyle="--", alpha=0.5, label="0.35m")
    ax.set_title("base_height [6]")
    ax.set_ylabel("m")
    ax.legend(fontsize=8)

    # 4. projected_gravity [7:10]
    ax = axes[1, 1]
    for i, label in enumerate(["gx", "gy", "gz"]):
        ax.plot(obs_t, obs[:, 7 + i], linewidth=0.8, label=label)
    ax.axhline(y=-1.0, color="red", linestyle="--", linewidth=0.5, alpha=0.6, label="gz=-1 (inverse)")
    ax.axhline(y=1.0, color="orange", linestyle="--", linewidth=0.5, alpha=0.6, label="gz=+1 (direct)")
    ax.set_title("projected_gravity [7:10] — KEY DIAGNOSTIC")
    ax.set_ylabel("normalized")
    ax.legend(fontsize=7)

    # 5. velocity_commands [10:13]
    ax = axes[2, 0]
    for i, label in enumerate(["cmd_vx", "cmd_vy", "cmd_wz"]):
        ax.plot(obs_t, obs[:, 10 + i], linewidth=0.8, label=label)
    ax.set_title("velocity_commands [10:13]")
    ax.set_ylabel("cmd value")
    ax.legend(fontsize=8)

    # 6. joint_pos [13:25] (Isaac Lab order, defaults subtracted)
    ax = axes[2, 1]
    for i in range(12):
        ax.plot(obs_t, obs[:, 13 + i], linewidth=0.5, alpha=0.6, label=ISAAC_JOINT_NAMES[i])
    ax.set_title("joint_pos [13:25] (default-subtracted, Isaac Lab order)")
    ax.set_ylabel("rad")
    ax.legend(fontsize=5, ncol=3, loc="upper right")

    # 7. joint_vel [25:37] (Isaac Lab order)
    ax = axes[3, 0]
    for i in range(12):
        ax.plot(obs_t, obs[:, 25 + i], linewidth=0.5, alpha=0.6, label=ISAAC_JOINT_NAMES[i])
    ax.set_title("joint_vel [25:37] (Isaac Lab order)")
    ax.set_ylabel("rad/s")
    ax.set_xlabel("time (s)")
    ax.legend(fontsize=5, ncol=3, loc="upper right")

    # 8. last_actions [37:49]
    ax = axes[3, 1]
    for i in range(12):
        ax.plot(obs_t, obs[:, 37 + i], linewidth=0.5, alpha=0.6, label=ISAAC_JOINT_NAMES[i])
    ax.set_title("last_actions [37:49]")
    ax.set_ylabel("raw action")
    ax.set_xlabel("time (s)")
    ax.legend(fontsize=5, ncol=3, loc="upper right")

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "03_observation_breakdown.png"), dpi=150)
    return fig


def plot_frequency_histogram(d, save_dir=None):
    """Plot 4: Frequency Histogram for actions topic."""
    if not _has(d, "actions_t"):
        print("Plot 4 (Frequency Histogram): SKIPPED — missing actions timestamps.")
        return

    act_t = d["actions_t"]
    if len(act_t) < 2:
        print("Plot 4 (Frequency Histogram): SKIPPED — too few samples.")
        return

    intervals = np.diff(act_t) * 1000.0  # Convert to ms.
    expected_ms = 40.0  # 25 Hz

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Plot 4: Action Publishing Frequency Analysis", fontsize=14, fontweight="bold")

    # Left: histogram of inter-message intervals.
    ax1.hist(intervals, bins=60, color="steelblue", alpha=0.7, edgecolor="none")
    ax1.axvline(x=expected_ms, color="red", linestyle="--", linewidth=1.5, label=f"expected {expected_ms:.0f}ms")
    ax1.set_xlabel("inter-message interval (ms)")
    ax1.set_ylabel("count")
    ax1.set_title("Action Interval Distribution")
    ax1.legend(fontsize=9)

    mean_hz = 1000.0 / np.mean(intervals) if np.mean(intervals) > 0 else 0.0
    jitter_ms = float(np.std(intervals))
    within_20pct = float(np.mean(np.abs(intervals - expected_ms) < 0.2 * expected_ms)) * 100.0
    stats_text = (
        f"mean Hz: {mean_hz:.1f}\n"
        f"mean interval: {np.mean(intervals):.1f}ms\n"
        f"jitter (std): {jitter_ms:.1f}ms\n"
        f"within 20% of target: {within_20pct:.0f}%\n"
        f"samples: {len(intervals)}"
    )
    ax1.text(
        0.98, 0.95, stats_text, transform=ax1.transAxes,
        fontsize=9, verticalalignment="top", horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    # Right: instantaneous frequency over time.
    inst_hz = 1000.0 / intervals
    mid_t = (act_t[:-1] + act_t[1:]) / 2.0
    ax2.plot(mid_t, inst_hz, linewidth=0.5, alpha=0.7, color="steelblue")
    ax2.axhline(y=25.0, color="red", linestyle="--", linewidth=1.0, label="25 Hz target")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("frequency (Hz)")
    ax2.set_title("Instantaneous Action Frequency")
    ax2.legend(fontsize=9)

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "04_frequency_histogram.png"), dpi=150)
    return fig


def plot_tracking_error(d, save_dir=None):
    """Plot 5: Per-joint tracking error (target vs actual)."""
    if not _has(d, "actions_t", "actions_data", "lowstate_t", "lowstate_joint_pos"):
        print("Plot 5 (Tracking Error): SKIPPED — missing actions or lowstate data.")
        return

    act_t = d["actions_t"]
    act_d = d["actions_data"]
    ls_t = d["lowstate_t"]
    ls_jp = d["lowstate_joint_pos"]

    # Only compare in overlapping time range.
    t_start = max(act_t[0], ls_t[0])
    t_end = min(act_t[-1], ls_t[-1])
    ls_mask = (ls_t >= t_start) & (ls_t <= t_end)
    if np.count_nonzero(ls_mask) < 10:
        print("Plot 5 (Tracking Error): SKIPPED — insufficient overlapping data.")
        return

    ls_t_m = ls_t[ls_mask]
    ls_jp_m = ls_jp[ls_mask]
    interp_targets = np.zeros_like(ls_jp_m)
    for j in range(12):
        interp_targets[:, j] = np.interp(ls_t_m, act_t, act_d[:, j])

    errors = np.abs(interp_targets - ls_jp_m)

    fig, axes = plt.subplots(4, 3, figsize=(16, 12), sharex=True)
    fig.suptitle("Plot 5: Tracking Error |target - actual|", fontsize=14, fontweight="bold")

    for row, (leg_name, indices) in enumerate(LEG_GROUPS):
        for col, (jtype, jidx) in enumerate(zip(JOINT_TYPE_NAMES, indices)):
            ax = axes[row, col]
            ax.plot(ls_t_m, errors[:, jidx], linewidth=0.5, alpha=0.8, color="darkorange")
            mean_e = float(np.mean(errors[:, jidx]))
            max_e = float(np.max(errors[:, jidx]))
            ax.axhline(y=mean_e, color="red", linestyle="--", linewidth=0.6, alpha=0.6)
            ax.set_title(f"{leg_name} {jtype} (mean={mean_e:.3f}, max={max_e:.3f})", fontsize=9)
            ax.set_ylabel("rad")
            if row == 3:
                ax.set_xlabel("time (s)")

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "05_tracking_error.png"), dpi=150)
    return fig


def plot_gravity_check(d, save_dir=None):
    """Plot 6: Gravity Convention Check."""
    if not _has(d, "obs_t", "obs_data"):
        print("Plot 6 (Gravity Check): SKIPPED — missing obs data.")
        return

    obs_t = d["obs_t"]
    obs = d["obs_data"]
    grav = obs[:, 7:10]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Plot 6: Gravity Convention Check", fontsize=14, fontweight="bold")

    # Left: gz over time with reference lines.
    ax1.plot(obs_t, grav[:, 2], linewidth=0.8, color="blue", label="gz")
    ax1.plot(obs_t, grav[:, 0], linewidth=0.5, alpha=0.5, color="gray", label="gx")
    ax1.plot(obs_t, grav[:, 1], linewidth=0.5, alpha=0.5, color="lightgray", label="gy")
    ax1.axhline(y=-1.0, color="green", linestyle="--", linewidth=1.5, label="gz=-1 (inverse, expected)")
    ax1.axhline(y=1.0, color="red", linestyle="--", linewidth=1.5, label="gz=+1 (direct, wrong)")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("gravity component")
    ax1.set_title("Projected Gravity Over Time")
    ax1.legend(fontsize=8)
    ax1.set_ylim(-1.5, 1.5)

    gz_mean = float(np.mean(grav[:, 2]))
    gz_std = float(np.std(grav[:, 2]))
    norm_mean = float(np.mean(np.linalg.norm(grav, axis=1)))
    if gz_mean < -0.8:
        verdict = "INVERSE (correct)"
        color = "green"
    elif gz_mean > 0.8:
        verdict = "DIRECT (WRONG!)"
        color = "red"
    else:
        verdict = "AMBIGUOUS"
        color = "orange"
    ax1.text(
        0.02, 0.02, f"mean gz={gz_mean:.3f} -> {verdict}",
        transform=ax1.transAxes, fontsize=10, fontweight="bold", color=color,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
    )

    # Right: gravity vector norm (should be ~1.0).
    norms = np.linalg.norm(grav, axis=1)
    ax2.plot(obs_t, norms, linewidth=0.8, color="purple")
    ax2.axhline(y=1.0, color="gray", linestyle="--", linewidth=1.0, label="unit norm")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("|gravity|")
    ax2.set_title("Gravity Vector Norm (should be ~1.0)")
    ax2.legend(fontsize=9)
    stats_text = (
        f"mean |g|: {norm_mean:.4f}\n"
        f"mean gz: {gz_mean:.4f}\n"
        f"gz std: {gz_std:.4f}"
    )
    ax2.text(
        0.98, 0.95, stats_text, transform=ax2.transAxes,
        fontsize=9, verticalalignment="top", horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "06_gravity_check.png"), dpi=150)
    return fig


def main():
    parser = argparse.ArgumentParser(
        description="Plot locomotion policy diagnostics from .npz recording."
    )
    parser.add_argument(
        "npz_file", type=str,
        help="Path to the .npz file produced by locomotion_diagnostics.py",
    )
    parser.add_argument(
        "--save-dir", type=str, default=None,
        help="If set, save plots as PNGs to this directory instead of showing interactively.",
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Do not call plt.show() (useful for headless/save-only mode).",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.npz_file):
        print(f"ERROR: File not found: {args.npz_file}", file=sys.stderr)
        return 1

    print(f"Loading {args.npz_file}...")
    d = dict(np.load(args.npz_file, allow_pickle=False))

    print(f"Arrays found: {sorted(d.keys())}")
    for k, v in sorted(d.items()):
        print(f"  {k}: shape={v.shape} dtype={v.dtype}")

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        print(f"Saving plots to {args.save_dir}/")

    plot_target_vs_actual(d, args.save_dir)
    plot_raw_action_analysis(d, args.save_dir)
    plot_observation_breakdown(d, args.save_dir)
    plot_frequency_histogram(d, args.save_dir)
    plot_tracking_error(d, args.save_dir)
    plot_gravity_check(d, args.save_dir)

    print("\nAll plots generated.")

    if not args.no_show and not args.save_dir:
        plt.show()
    elif args.save_dir:
        print(f"Plots saved to {args.save_dir}/")
        plt.close("all")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
