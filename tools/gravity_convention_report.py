#!/usr/bin/env python3
"""Offline gravity-convention scorer from rosbag /lowstate.

Determines whether projected gravity should use:
  - direct:  R(q) @ [0, 0, -1]
  - inverse: R(q)^-1 @ [0, 0, -1]

It compares both candidates against IMU accelerometer direction with sign
ambiguity (a and -a), then reports per-candidate mean angular errors,
winner, confidence ratio, and quasi-static sample counts.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot

try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "Failed to import ROS2 bag/message libraries. "
        "Source your ROS 2 environment first, e.g.\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source ~/ros2_ws/install/setup.bash\n"
        f"Import error: {exc}"
    )


WORLD_GRAVITY = np.array([0.0, 0.0, -1.0], dtype=np.float64)


@dataclass
class SweepResult:
    gyro_max: float
    quasi_count: int
    quasi_ratio: float
    means: Dict[str, float]
    best_direct_error: float
    best_direct_sign: str
    best_inverse_error: float
    best_inverse_sign: str
    winner: str
    confidence_ratio: float


def _normalize(vec: np.ndarray) -> Tuple[np.ndarray | None, float]:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        return None, norm
    return (vec / norm).astype(np.float64), norm


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    dot_value = float(np.dot(a, b))
    dot_value = max(-1.0, min(1.0, dot_value))
    return math.degrees(math.acos(dot_value))


def _parse_float_list(text: str) -> List[float]:
    if not text.strip():
        return []
    values = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        values.append(float(token))
    return values


def _scan_bag(
    bag_dir: str,
    topic: str,
    storage_id: str,
) -> Tuple[int, Dict[str, np.ndarray]]:
    """Scan bag once and cache per-sample metrics."""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id=storage_id),
        ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )

    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise RuntimeError(f'Topic "{topic}" not found in bag. Available topics: {sorted(topic_types.keys())}')

    msg_type = get_message(topic_types[topic])

    gyro_norms: List[float] = []
    accel_norms: List[float] = []
    direct_pos_errs: List[float] = []
    direct_neg_errs: List[float] = []
    inverse_pos_errs: List[float] = []
    inverse_neg_errs: List[float] = []

    lowstate_total = 0
    while reader.has_next():
        next_topic, raw_data, _ = reader.read_next()
        if next_topic != topic:
            continue
        lowstate_total += 1

        msg = deserialize_message(raw_data, msg_type)
        quat_wxyz = np.array(msg.imu_state.quaternion[0:4], dtype=np.float64)
        gyro = np.array(msg.imu_state.gyroscope[0:3], dtype=np.float64)
        accel = np.array(msg.imu_state.accelerometer[0:3], dtype=np.float64)

        accel_unit, accel_norm = _normalize(accel)
        if accel_unit is None:
            continue

        try:
            rot = Rot.from_quat(
                [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
            )
        except Exception:
            continue

        direct = rot.as_matrix() @ WORLD_GRAVITY
        inverse = rot.inv().as_matrix() @ WORLD_GRAVITY
        direct_unit, _ = _normalize(direct)
        inverse_unit, _ = _normalize(inverse)
        if direct_unit is None or inverse_unit is None:
            continue

        accel_neg = -accel_unit

        gyro_norms.append(float(np.linalg.norm(gyro)))
        accel_norms.append(accel_norm)
        direct_pos_errs.append(_angle_deg(direct_unit, accel_unit))
        direct_neg_errs.append(_angle_deg(direct_unit, accel_neg))
        inverse_pos_errs.append(_angle_deg(inverse_unit, accel_unit))
        inverse_neg_errs.append(_angle_deg(inverse_unit, accel_neg))

    cached = {
        "gyro_norm": np.asarray(gyro_norms, dtype=np.float64),
        "accel_norm": np.asarray(accel_norms, dtype=np.float64),
        "direct_pos": np.asarray(direct_pos_errs, dtype=np.float64),
        "direct_neg": np.asarray(direct_neg_errs, dtype=np.float64),
        "inverse_pos": np.asarray(inverse_pos_errs, dtype=np.float64),
        "inverse_neg": np.asarray(inverse_neg_errs, dtype=np.float64),
    }
    return lowstate_total, cached


def _evaluate_threshold(
    cached: Dict[str, np.ndarray],
    total_lowstate_count: int,
    gyro_max: float,
    accel_min: float,
    accel_max: float,
) -> SweepResult:
    mask = (
        (cached["gyro_norm"] <= gyro_max)
        & (cached["accel_norm"] >= accel_min)
        & (cached["accel_norm"] <= accel_max)
    )
    quasi_count = int(np.count_nonzero(mask))
    if quasi_count <= 0:
        raise RuntimeError(
            f"No quasi-static samples for gyro_max={gyro_max:.3f}, "
            f"accel_norm in [{accel_min:.3f}, {accel_max:.3f}]."
        )

    means = {
        key: float(np.mean(values[mask]))
        for key, values in cached.items()
        if key in ("direct_pos", "direct_neg", "inverse_pos", "inverse_neg")
    }

    if means["direct_pos"] <= means["direct_neg"]:
        best_direct_error = means["direct_pos"]
        best_direct_sign = "+"
    else:
        best_direct_error = means["direct_neg"]
        best_direct_sign = "-"

    if means["inverse_pos"] <= means["inverse_neg"]:
        best_inverse_error = means["inverse_pos"]
        best_inverse_sign = "+"
    else:
        best_inverse_error = means["inverse_neg"]
        best_inverse_sign = "-"

    if best_direct_error <= best_inverse_error:
        winner = "direct"
        confidence_ratio = best_inverse_error / max(best_direct_error, 1e-9)
    else:
        winner = "inverse"
        confidence_ratio = best_direct_error / max(best_inverse_error, 1e-9)

    return SweepResult(
        gyro_max=gyro_max,
        quasi_count=quasi_count,
        quasi_ratio=quasi_count / float(max(1, total_lowstate_count)),
        means=means,
        best_direct_error=best_direct_error,
        best_direct_sign=best_direct_sign,
        best_inverse_error=best_inverse_error,
        best_inverse_sign=best_inverse_sign,
        winner=winner,
        confidence_ratio=confidence_ratio,
    )


def _build_gyro_sweep(primary: float, extra_values: Sequence[float]) -> List[float]:
    values = [primary]
    for value in extra_values:
        if value not in values:
            values.append(value)
    # Keep descending by strictness: high threshold first.
    return sorted(values, reverse=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute gravity projection convention from /lowstate rosbag."
    )
    parser.add_argument("--bag", required=True, help="Rosbag directory path")
    parser.add_argument("--topic", default="/lowstate", help="LowState topic")
    parser.add_argument("--storage-id", default="sqlite3", help="Bag storage id (sqlite3/mcap)")
    parser.add_argument("--gyro-max", type=float, default=0.25, help="Primary quasi-static gyro norm gate")
    parser.add_argument("--accel-min", type=float, default=7.0, help="Minimum accel norm gate")
    parser.add_argument("--accel-max", type=float, default=12.0, help="Maximum accel norm gate")
    parser.add_argument(
        "--gyro-sweep",
        default="0.15,0.10",
        help=(
            "Additional stricter gyro thresholds for stability check, comma-separated. "
            "Use empty string to disable."
        ),
    )
    parser.add_argument(
        "--min-confidence-ratio",
        type=float,
        default=2.0,
        help="Primary acceptance threshold for confidence ratio",
    )
    parser.add_argument(
        "--min-quasi-samples",
        type=int,
        default=3000,
        help="Minimum quasi-static samples for data quality acceptance",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sweep_values = _build_gyro_sweep(args.gyro_max, _parse_float_list(args.gyro_sweep))

    total_count, cached = _scan_bag(args.bag, args.topic, args.storage_id)
    if total_count <= 0:
        print("ERROR: no messages found on the requested topic.", file=sys.stderr)
        return 2

    print(f"Bag: {args.bag}")
    print(f"Topic: {args.topic}")
    print(f"Total {args.topic} samples: {total_count}")
    print(
        f"Quasi-static accel gate: [{args.accel_min:.3f}, {args.accel_max:.3f}] m/s^2"
    )

    results: List[SweepResult] = []
    for gyro_threshold in sweep_values:
        try:
            result = _evaluate_threshold(
                cached=cached,
                total_lowstate_count=total_count,
                gyro_max=gyro_threshold,
                accel_min=args.accel_min,
                accel_max=args.accel_max,
            )
        except RuntimeError as exc:
            print(f"\nGyro<= {gyro_threshold:.3f}: ERROR: {exc}")
            continue

        results.append(result)
        print(f"\nGyro<= {result.gyro_max:.3f}")
        print(
            f"  quasi_static_samples={result.quasi_count} "
            f"(ratio={result.quasi_ratio:.3f})"
        )
        print(
            "  mean_err_deg: "
            f"direct(+/−)=({result.means['direct_pos']:.3f}, {result.means['direct_neg']:.3f}) "
            f"inverse(+/−)=({result.means['inverse_pos']:.3f}, {result.means['inverse_neg']:.3f})"
        )
        print(
            "  best: "
            f"direct{result.best_direct_sign}={result.best_direct_error:.3f} "
            f"inverse{result.best_inverse_sign}={result.best_inverse_error:.3f}"
        )
        print(
            f"  winner={result.winner} confidence_ratio={result.confidence_ratio:.3f}x"
        )

    if not results:
        print("\nERROR: no valid sweep results.", file=sys.stderr)
        return 2

    primary = next((r for r in results if abs(r.gyro_max - args.gyro_max) < 1e-12), results[0])
    winners = {r.winner for r in results}
    stable_winner = len(winners) == 1
    ratio_pass = primary.confidence_ratio >= args.min_confidence_ratio
    quasi_pass = primary.quasi_count >= args.min_quasi_samples

    print("\nAcceptance")
    print(
        f"  primary_confidence_ratio >= {args.min_confidence_ratio:.3f}: "
        f"{'PASS' if ratio_pass else 'FAIL'} "
        f"({primary.confidence_ratio:.3f}x)"
    )
    print(
        f"  primary_quasi_samples >= {args.min_quasi_samples}: "
        f"{'PASS' if quasi_pass else 'FAIL'} "
        f"({primary.quasi_count})"
    )
    print(
        "  winner stable across gyro sweep: "
        f"{'PASS' if stable_winner else 'FAIL'} "
        f"({', '.join(sorted(winners))})"
    )
    print(
        f"\nRecommended gravity_projection_mode: {primary.winner}"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
