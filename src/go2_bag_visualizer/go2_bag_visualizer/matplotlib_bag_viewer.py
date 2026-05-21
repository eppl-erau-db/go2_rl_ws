#!/usr/bin/env python3
"""Offline 3D matplotlib visualizer for Go2 rosbag2 data."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from lidar_obstacle_detection_msgs.msg import ObstacleList
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState
from visualization_msgs.msg import Marker, MarkerArray

from go2_bag_visualizer.bag_finder import DEFAULT_DATE_SUBSTRING, find_dated_bags


LEG_NAMES = ('FR', 'FL', 'RR', 'RL')
JOINT_GROUPS = (
    ('hip', (0, 3, 6, 9)),
    ('thigh', (1, 4, 7, 10)),
    ('calf', (2, 5, 8, 11)),
)

FR_HIP_OFFSET = np.array([0.247, -0.050, 0.0], dtype=float)
ABAD_OFFSET = 0.083
THIGH_LENGTH = 0.210
CALF_LENGTH = 0.210

EDGE_INDICES = (
    (0, 1),
    (0, 2),
    (0, 4),
    (1, 3),
    (1, 5),
    (2, 3),
    (2, 6),
    (3, 7),
    (4, 5),
    (4, 6),
    (5, 7),
    (6, 7),
)


@dataclass
class BagSamples:
    odom_times: List[int] = field(default_factory=list)
    odom_positions: List[Tuple[float, float, float]] = field(default_factory=list)
    odom_quaternions: List[Tuple[float, float, float, float]] = field(
        default_factory=list
    )
    nav_goal_times: List[int] = field(default_factory=list)
    nav_goal_vectors: List[Tuple[float, float, float, float]] = field(
        default_factory=list
    )
    nav_goal_marker_times: List[int] = field(default_factory=list)
    nav_goal_marker_points: List[Tuple[float, float, float]] = field(
        default_factory=list
    )
    nav_goal_marker_frames: List[str] = field(default_factory=list)
    nav_goal_arrow_times: List[int] = field(default_factory=list)
    nav_goal_arrow_starts: List[Tuple[float, float, float]] = field(
        default_factory=list
    )
    nav_goal_arrow_ends: List[Tuple[float, float, float]] = field(default_factory=list)
    nav_goal_arrow_frames: List[str] = field(default_factory=list)
    goal_pose_times: List[int] = field(default_factory=list)
    goal_pose_points: List[Tuple[float, float, float]] = field(default_factory=list)
    goal_pose_quaternions: List[Tuple[float, float, float, float]] = field(
        default_factory=list
    )
    goal_pose_frames: List[str] = field(default_factory=list)
    obstacle_times: List[int] = field(default_factory=list)
    obstacle_centers_base: List[Tuple[float, float, float]] = field(
        default_factory=list
    )
    obstacle_sizes: List[Tuple[float, float, float]] = field(default_factory=list)
    surface_normal_times: List[int] = field(default_factory=list)
    surface_normals: List[Tuple[float, float, float]] = field(default_factory=list)
    surface_normal_frames: List[str] = field(default_factory=list)
    closest_times: List[int] = field(default_factory=list)
    closest_points_base: List[Tuple[float, float, float]] = field(default_factory=list)
    object_times: List[int] = field(default_factory=list)
    object_centers_base: List[Tuple[float, float, float]] = field(default_factory=list)
    object_contact_points_base: List[Tuple[float, float, float]] = field(
        default_factory=list
    )
    pose_start_time: Optional[int] = None
    pose_end_time: Optional[int] = None
    pose_times: List[int] = field(default_factory=list)
    pose_points_base: List[Tuple[float, float, float]] = field(default_factory=list)
    lowstate_times: List[int] = field(default_factory=list)
    foot_forces: List[Tuple[float, float, float, float]] = field(default_factory=list)
    foot_force_est: List[Tuple[float, float, float, float]] = field(
        default_factory=list
    )
    joint_positions: List[Tuple[float, ...]] = field(default_factory=list)


@dataclass
class MetricsConfig:
    touch_force_threshold: float
    ee_ground_clearance: float
    pose_reach_tolerance: float
    pose_command_calibration_sec: float


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _message_stamp_or_bag_time(msg, bag_time_ns: int) -> int:
    header = getattr(msg, 'header', None)
    if header is None:
        return int(bag_time_ns)
    stamp_ns = _stamp_to_ns(header.stamp)
    return stamp_ns if stamp_ns > 0 else int(bag_time_ns)


def _point_tuple(point) -> Tuple[float, float, float]:
    return (float(point.x), float(point.y), float(point.z))


def _finite_tuple(point: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in point)


def _finite_point(point) -> bool:
    return (
        math.isfinite(float(point.x))
        and math.isfinite(float(point.y))
        and math.isfinite(float(point.z))
    )


def _finite_vector(vector) -> bool:
    return (
        math.isfinite(float(vector.x))
        and math.isfinite(float(vector.y))
        and math.isfinite(float(vector.z))
    )


def _vector_tuple(vector) -> Tuple[float, float, float]:
    return (float(vector.x), float(vector.y), float(vector.z))


def _nav_goal_vector(
    msg: Float32MultiArray,
) -> Optional[Tuple[float, float, float, float]]:
    try:
        values = [float(value) for value in msg.data[:4]]
    except (TypeError, ValueError):
        return None

    if len(values) < 3:
        return None
    while len(values) < 4:
        values.append(0.0)
    if not _finite_tuple(values):
        return None
    return (values[0], values[1], values[2], values[3])


def _nav_goal_marker(msg: MarkerArray) -> Optional[Marker]:
    candidates = [
        marker
        for marker in msg.markers
        if marker.action == Marker.ADD
        and marker.type == Marker.SPHERE
        and 'nav_goal' in marker.ns
        and _finite_point(marker.pose.position)
    ]
    if not candidates:
        candidates = [
            marker
            for marker in msg.markers
            if marker.action == Marker.ADD
            and 'nav_goal' in marker.ns
            and _finite_point(marker.pose.position)
        ]
    return candidates[0] if candidates else None


def _nav_goal_arrow_marker(msg: MarkerArray) -> Optional[Marker]:
    candidates = [
        marker
        for marker in msg.markers
        if marker.action == Marker.ADD
        and marker.type == Marker.ARROW
        and 'nav_goal' in marker.ns
        and len(marker.points) >= 2
        and _finite_point(marker.points[0])
        and _finite_point(marker.points[1])
    ]
    return candidates[0] if candidates else None


def _quaternion_to_matrix(quat_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return np.eye(3)

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def _quaternion_to_rpy(quat_xyzw: Sequence[float]) -> Tuple[float, float, float]:
    rotation = _quaternion_to_matrix(quat_xyzw)
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    pitch = math.atan2(
        -rotation[2, 0],
        math.sqrt(rotation[2, 1] * rotation[2, 1] + rotation[2, 2] * rotation[2, 2]),
    )
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return roll, pitch, yaw


def _yaw_from_quaternion(quat_xyzw: Sequence[float]) -> float:
    return _quaternion_to_rpy(quat_xyzw)[2]


def _angle_diff_rad(a: float, b: float) -> float:
    return math.atan2(math.sin(float(a) - float(b)), math.cos(float(a) - float(b)))


def _rotation_x(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def _rotation_y(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


def _fr_foot_position_from_joints(joint_positions: Sequence[float]) -> np.ndarray:
    q_abad, q_hip, q_knee = (float(v) for v in joint_positions[:3])
    p_ab = np.array([0.0, -ABAD_OFFSET, 0.0], dtype=float)
    p_th = np.array([0.0, 0.0, -THIGH_LENGTH], dtype=float)
    p_cf = np.array([0.0, 0.0, -CALF_LENGTH], dtype=float)
    return FR_HIP_OFFSET + _rotation_x(q_abad) @ (
        p_ab + _rotation_y(q_hip) @ (p_th + _rotation_y(q_knee) @ p_cf)
    )


def _nearest_odom_indices(
    sample_times: np.ndarray,
    odom_times: np.ndarray,
) -> np.ndarray:
    right = np.searchsorted(odom_times, sample_times, side='left')
    right = np.clip(right, 0, len(odom_times) - 1)
    left = np.clip(right - 1, 0, len(odom_times) - 1)
    choose_left = np.abs(sample_times - odom_times[left]) <= np.abs(
        odom_times[right] - sample_times
    )
    return np.where(choose_left, left, right)


def _transform_base_points_to_odom(
    sample_times: Sequence[int],
    points_base: Sequence[Tuple[float, float, float]],
    odom_times: np.ndarray,
    odom_positions: np.ndarray,
    odom_quaternions: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    if not sample_times or not points_base:
        return np.empty((0, 3), dtype=float), np.empty((0,), dtype=int)

    sample_times_arr = np.asarray(sample_times, dtype=np.int64)
    points = np.asarray(points_base, dtype=float)
    odom_indices = _nearest_odom_indices(sample_times_arr, odom_times)

    transformed = np.empty_like(points)
    for out_idx, odom_idx in enumerate(odom_indices):
        rotation = _quaternion_to_matrix(odom_quaternions[odom_idx])
        transformed[out_idx] = odom_positions[odom_idx] + rotation @ points[out_idx]

    return transformed, odom_indices


def _same_frame(frame_id: str, expected: str) -> bool:
    return frame_id.strip().strip('/') == expected.strip().strip('/')


def _transform_points_to_odom(
    sample_times: Sequence[int],
    points: Sequence[Tuple[float, float, float]],
    frame_ids: Sequence[str],
    odom_times: np.ndarray,
    odom_positions: np.ndarray,
    odom_quaternions: np.ndarray,
    fixed_frame: str = 'odom',
    base_frame: str = 'base_link',
) -> np.ndarray:
    if len(sample_times) == 0 or len(points) == 0:
        return np.empty((0, 3), dtype=float)

    transformed = []
    for sample_time, point, frame_id in zip(sample_times, points, frame_ids):
        point_arr = np.asarray(point, dtype=float)
        if _same_frame(frame_id, fixed_frame):
            transformed.append(point_arr)
            continue

        if _same_frame(frame_id, base_frame) or not frame_id:
            odom_idx = int(
                _nearest_odom_indices(
                    np.asarray([sample_time], dtype=np.int64),
                    odom_times,
                )[0]
            )
            rotation = _quaternion_to_matrix(odom_quaternions[odom_idx])
            transformed.append(odom_positions[odom_idx] + rotation @ point_arr)
            continue

        transformed.append(point_arr)

    return np.asarray(transformed, dtype=float)


def _transform_vectors_to_odom(
    sample_times: Sequence[int],
    vectors: Sequence[Tuple[float, float, float]],
    frame_ids: Sequence[str],
    odom_times: np.ndarray,
    odom_quaternions: np.ndarray,
    fixed_frame: str = 'odom',
    base_frame: str = 'base_link',
) -> np.ndarray:
    if len(sample_times) == 0 or len(vectors) == 0:
        return np.empty((0, 3), dtype=float)

    transformed = []
    for sample_time, vector, frame_id in zip(sample_times, vectors, frame_ids):
        vector_arr = np.asarray(vector, dtype=float)
        if _same_frame(frame_id, fixed_frame):
            transformed.append(vector_arr)
            continue

        if _same_frame(frame_id, base_frame) or not frame_id:
            odom_idx = int(
                _nearest_odom_indices(
                    np.asarray([sample_time], dtype=np.int64),
                    odom_times,
                )[0]
            )
            transformed.append(
                _quaternion_to_matrix(odom_quaternions[odom_idx]) @ vector_arr
            )
            continue

        transformed.append(vector_arr)

    return np.asarray(transformed, dtype=float)


def _time_window_mask(
    sample_times: Sequence[int],
    start_ns: Optional[int] = None,
    end_ns: Optional[int] = None,
) -> np.ndarray:
    times = np.asarray(sample_times, dtype=np.int64)
    mask = np.ones((len(times),), dtype=bool)
    if start_ns is not None:
        mask &= times >= int(start_ns)
    if end_ns is not None:
        mask &= times <= int(end_ns)
    return mask


def _normalized_times(
    sample_times: Sequence[int],
    start_ns: int,
    end_ns: int,
) -> np.ndarray:
    if len(sample_times) == 0:
        return np.empty((0,), dtype=float)
    duration = max(float(end_ns - start_ns), 1.0)
    return (np.asarray(sample_times, dtype=float) - float(start_ns)) / duration


def _topic_counts(bag_path: Path) -> Dict[str, int]:
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        return {}

    metadata = yaml.safe_load(metadata_path.read_text())
    topics = metadata['rosbag2_bagfile_information']['topics_with_message_count']
    return {
        entry['topic_metadata']['name']: int(entry['message_count'])
        for entry in topics
    }


def _valid_bags(
    candidates: Iterable[Path],
    odom_topic: str,
    obstacle_topic: str,
    pose_command_topic: str,
) -> List[Path]:
    bags = []
    for bag_path in candidates:
        counts = _topic_counts(bag_path)
        if counts.get(odom_topic, 0) <= 0:
            print(f'[skip] {bag_path}: no {odom_topic} messages')
            continue
        if (
            counts.get(obstacle_topic, 0) <= 0
            and counts.get(pose_command_topic, 0) <= 0
        ):
            print(
                f'[skip] {bag_path}: no {obstacle_topic} or '
                f'{pose_command_topic} messages'
            )
            continue
        bags.append(bag_path)
    return bags


def read_bag_samples(
    bag_path: Path,
    odom_topic: str,
    nav_goal_topic: str,
    nav_goal_marker_topic: str,
    obstacle_topic: str,
    pose_command_topic: str,
    lowstate_topic: str,
    nav_goal_stride: int,
    pose_stride: int,
    obstacle_list_stride: int,
    lowstate_stride: int,
    max_obstacle_lists: Optional[int],
    goal_pose_topic: str = '/goal_pose',
) -> BagSamples:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )

    samples = BagSamples()
    nav_goal_seen = 0
    nav_goal_markers_seen = 0
    pose_seen = 0
    obstacle_lists_seen = 0
    obstacle_lists_used = 0
    lowstate_seen = 0

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()

        if topic == odom_topic:
            msg = deserialize_message(data, Odometry)
            pose = msg.pose.pose
            samples.odom_times.append(_message_stamp_or_bag_time(msg, bag_time_ns))
            samples.odom_positions.append(_point_tuple(pose.position))
            samples.odom_quaternions.append(
                (
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                )
            )

        elif topic == nav_goal_topic:
            nav_goal_seen += 1
            if nav_goal_seen % nav_goal_stride != 0:
                continue
            msg = deserialize_message(data, Float32MultiArray)
            goal = _nav_goal_vector(msg)
            if goal is not None:
                samples.nav_goal_times.append(int(bag_time_ns))
                samples.nav_goal_vectors.append(goal)

        elif topic == nav_goal_marker_topic:
            nav_goal_markers_seen += 1
            if nav_goal_markers_seen % nav_goal_stride != 0:
                continue
            msg = deserialize_message(data, MarkerArray)
            marker = _nav_goal_marker(msg)
            if marker is not None:
                samples.nav_goal_marker_times.append(
                    _message_stamp_or_bag_time(marker, bag_time_ns)
                )
                samples.nav_goal_marker_points.append(
                    _point_tuple(marker.pose.position)
                )
                samples.nav_goal_marker_frames.append(marker.header.frame_id)
            arrow = _nav_goal_arrow_marker(msg)
            if arrow is not None:
                samples.nav_goal_arrow_times.append(
                    _message_stamp_or_bag_time(arrow, bag_time_ns)
                )
                samples.nav_goal_arrow_starts.append(_point_tuple(arrow.points[0]))
                samples.nav_goal_arrow_ends.append(_point_tuple(arrow.points[1]))
                samples.nav_goal_arrow_frames.append(arrow.header.frame_id)

        elif topic == goal_pose_topic:
            msg = deserialize_message(data, PoseStamped)
            samples.goal_pose_times.append(_message_stamp_or_bag_time(msg, bag_time_ns))
            samples.goal_pose_points.append(_point_tuple(msg.pose.position))
            samples.goal_pose_quaternions.append(
                (
                    float(msg.pose.orientation.x),
                    float(msg.pose.orientation.y),
                    float(msg.pose.orientation.z),
                    float(msg.pose.orientation.w),
                )
            )
            samples.goal_pose_frames.append(msg.header.frame_id)

        elif topic == pose_command_topic:
            pose_seen += 1
            pose_time_ns = int(bag_time_ns)
            if samples.pose_start_time is None:
                samples.pose_start_time = pose_time_ns
            samples.pose_end_time = pose_time_ns
            if pose_seen % pose_stride != 0:
                continue
            msg = deserialize_message(data, Pose)
            samples.pose_times.append(pose_time_ns)
            samples.pose_points_base.append(_point_tuple(msg.position))

        elif topic == obstacle_topic:
            obstacle_lists_seen += 1
            if obstacle_lists_seen % obstacle_list_stride != 0:
                continue
            if (
                max_obstacle_lists is not None
                and obstacle_lists_used >= max_obstacle_lists
            ):
                continue

            msg = deserialize_message(data, ObstacleList)
            msg_time_ns = _message_stamp_or_bag_time(msg, bag_time_ns)
            obstacle_lists_used += 1

            if msg.obstacles:
                primary_obstacle = msg.obstacles[0]
                object_center = _point_tuple(primary_obstacle.position)
                samples.object_times.append(msg_time_ns)
                samples.object_centers_base.append(object_center)
                if _finite_point(primary_obstacle.closest_surface_point):
                    samples.object_contact_points_base.append(
                        _point_tuple(primary_obstacle.closest_surface_point)
                    )
                else:
                    samples.object_contact_points_base.append(object_center)

            for obstacle in msg.obstacles:
                samples.obstacle_times.append(msg_time_ns)
                samples.obstacle_centers_base.append(_point_tuple(obstacle.position))
                samples.obstacle_sizes.append(
                    (
                        float(obstacle.width),
                        float(obstacle.height),
                        float(obstacle.length),
                    )
                )
                for normal in obstacle.surface_normals:
                    if _finite_vector(normal):
                        samples.surface_normal_times.append(msg_time_ns)
                        samples.surface_normals.append(_vector_tuple(normal))
                        samples.surface_normal_frames.append(msg.header.frame_id)
                if _finite_point(obstacle.closest_surface_point):
                    samples.closest_times.append(msg_time_ns)
                    samples.closest_points_base.append(
                        _point_tuple(obstacle.closest_surface_point)
                    )

        elif topic == lowstate_topic:
            lowstate_seen += 1
            if lowstate_seen % lowstate_stride != 0:
                continue
            msg = deserialize_message(data, LowState)
            samples.lowstate_times.append(int(bag_time_ns))
            samples.foot_forces.append(tuple(float(v) for v in msg.foot_force))
            samples.foot_force_est.append(tuple(float(v) for v in msg.foot_force_est))
            samples.joint_positions.append(
                tuple(float(motor.q) for motor in msg.motor_state[:12])
            )

    return samples


def _sort_odom(samples: BagSamples) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    odom_times = np.asarray(samples.odom_times, dtype=np.int64)
    odom_positions = np.asarray(samples.odom_positions, dtype=float)
    odom_quaternions = np.asarray(samples.odom_quaternions, dtype=float)
    order = np.argsort(odom_times)
    return odom_times[order], odom_positions[order], odom_quaternions[order]


def _time_colors(sample_times: Sequence[int], odom_times: np.ndarray) -> np.ndarray:
    if not sample_times:
        return np.empty((0,), dtype=float)
    start = float(odom_times[0])
    duration = max(float(odom_times[-1] - odom_times[0]), 1.0)
    return (np.asarray(sample_times, dtype=float) - start) / duration


def _box_segments(
    centers_base: Sequence[Tuple[float, float, float]],
    sizes: Sequence[Tuple[float, float, float]],
    odom_indices: np.ndarray,
    odom_positions: np.ndarray,
    odom_quaternions: np.ndarray,
    box_stride: int,
) -> List[np.ndarray]:
    segments: List[np.ndarray] = []

    for sample_idx in range(0, len(centers_base), box_stride):
        center = np.asarray(centers_base[sample_idx], dtype=float)
        size = np.maximum(np.asarray(sizes[sample_idx], dtype=float), 0.01)
        half = size * 0.5
        offsets = np.array(
            [
                [-half[0], -half[1], -half[2]],
                [-half[0], -half[1], half[2]],
                [-half[0], half[1], -half[2]],
                [-half[0], half[1], half[2]],
                [half[0], -half[1], -half[2]],
                [half[0], -half[1], half[2]],
                [half[0], half[1], -half[2]],
                [half[0], half[1], half[2]],
            ],
            dtype=float,
        )
        odom_idx = int(odom_indices[sample_idx])
        rotation = _quaternion_to_matrix(odom_quaternions[odom_idx])
        corners = odom_positions[odom_idx] + (rotation @ (center + offsets).T).T
        for edge_start, edge_end in EDGE_INDICES:
            segments.append(np.vstack((corners[edge_start], corners[edge_end])))

    return segments


def _set_axes_equal(ax) -> None:
    x_limits = np.asarray(ax.get_xlim3d(), dtype=float)
    y_limits = np.asarray(ax.get_ylim3d(), dtype=float)
    z_limits = np.asarray(ax.get_zlim3d(), dtype=float)

    ranges = np.array(
        [
            abs(x_limits[1] - x_limits[0]),
            abs(y_limits[1] - y_limits[0]),
            abs(z_limits[1] - z_limits[0]),
        ],
        dtype=float,
    )
    centers = np.array(
        [
            np.mean(x_limits),
            np.mean(y_limits),
            np.mean(z_limits),
        ],
        dtype=float,
    )
    radius = max(np.max(ranges) * 0.5, 0.5)

    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def _seconds_from_start(times_ns: np.ndarray, start_ns: int) -> np.ndarray:
    return (times_ns.astype(float) - float(start_ns)) * 1e-9


def _pose_window_mask(
    lowstate_times: np.ndarray,
    pose_times: Sequence[int],
    padding_sec: float,
) -> Tuple[np.ndarray, Optional[int], Optional[int]]:
    if len(lowstate_times) == 0 or not pose_times:
        return np.zeros((len(lowstate_times),), dtype=bool), None, None

    pose_times_arr = np.asarray(pose_times, dtype=np.int64)
    pad_ns = int(max(float(padding_sec), 0.0) * 1e9)
    window_start = int(pose_times_arr[0]) - pad_ns
    window_end = int(pose_times_arr[-1]) + pad_ns
    return (
        (lowstate_times >= window_start) & (lowstate_times <= window_end),
        window_start,
        window_end,
    )


def plot_lowstate_during_pose_commands(
    bag_path: Path,
    samples: BagSamples,
    save_dir: Optional[Path],
    show: bool,
    pose_window_padding_sec: float,
) -> None:
    if not samples.pose_times:
        print(f'[skip] {bag_path}: no /pose_command samples for lowstate plot')
        return
    if not samples.lowstate_times:
        print(f'[skip] {bag_path}: no lowstate samples')
        return

    lowstate_times = np.asarray(samples.lowstate_times, dtype=np.int64)
    foot_forces = np.asarray(samples.foot_forces, dtype=float)
    foot_force_est = np.asarray(samples.foot_force_est, dtype=float)
    joint_positions = np.asarray(samples.joint_positions, dtype=float)

    mask, window_start, window_end = _pose_window_mask(
        lowstate_times,
        samples.pose_times,
        pose_window_padding_sec,
    )
    if window_start is None or window_end is None or not np.any(mask):
        print(f'[skip] {bag_path}: no lowstate samples inside /pose_command window')
        return

    t = _seconds_from_start(lowstate_times[mask], window_start)
    pose_t = _seconds_from_start(np.asarray(samples.pose_times), window_start)
    foot_forces = foot_forces[mask]
    foot_force_est = foot_force_est[mask]
    joint_positions = joint_positions[mask]

    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(f'{bag_path.name}: lowstate while /pose_command is active')

    for leg_idx, leg_name in enumerate(LEG_NAMES):
        axes[0].plot(
            t,
            foot_forces[:, leg_idx],
            linewidth=1.0,
            label=f'{leg_name} force',
        )
        if np.any(np.abs(foot_force_est[:, leg_idx]) > 1e-6):
            axes[0].plot(
                t,
                foot_force_est[:, leg_idx],
                linestyle='--',
                linewidth=0.8,
                alpha=0.7,
                label=f'{leg_name} est',
            )
    axes[0].set_ylabel('foot force')
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc='upper right', ncol=4, fontsize='small')

    for axis, (joint_name, indices) in zip(axes[1:], JOINT_GROUPS):
        for leg_name, joint_idx in zip(LEG_NAMES, indices):
            axis.plot(
                t,
                joint_positions[:, joint_idx],
                linewidth=1.0,
                label=f'{leg_name}_{joint_name}',
            )
        axis.set_ylabel(f'{joint_name} q [rad]')
        axis.grid(True, alpha=0.25)
        axis.legend(loc='upper right', ncol=4, fontsize='small')

    for axis in axes:
        axis.axvline(
            pose_t[0],
            color='tab:purple',
            linestyle=':',
            linewidth=1.0,
            alpha=0.9,
        )
        axis.axvline(
            pose_t[-1],
            color='tab:purple',
            linestyle=':',
            linewidth=1.0,
            alpha=0.9,
        )
        ymin, ymax = axis.get_ylim()
        axis.vlines(
            pose_t,
            ymin,
            ymin + 0.04 * (ymax - ymin),
            color='tab:purple',
            linewidth=0.25,
            alpha=0.18,
        )

    axes[-1].set_xlabel('seconds from /pose_command window start')
    fig.tight_layout()

    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)
        output_path = save_dir / f'{bag_path.name}_lowstate_pose_command.png'
        fig.savefig(output_path, dpi=180)
        print(f'[saved] {output_path}')

    if show:
        print(f'[show] {bag_path}; close lowstate figure to continue')
        plt.show()
    else:
        plt.close(fig)


def _nav_goal_points_odom(
    samples: BagSamples,
    odom_times: np.ndarray,
    odom_positions: np.ndarray,
    odom_quaternions: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, str]:
    if samples.nav_goal_marker_points:
        return (
            _transform_points_to_odom(
                samples.nav_goal_marker_times,
                samples.nav_goal_marker_points,
                samples.nav_goal_marker_frames,
                odom_times,
                odom_positions,
                odom_quaternions,
            ),
            np.asarray(samples.nav_goal_marker_times, dtype=np.int64),
            '/nav_goal_debug_markers',
        )

    if samples.nav_goal_vectors:
        return (
            np.asarray([goal[:3] for goal in samples.nav_goal_vectors], dtype=float),
            np.asarray(samples.nav_goal_times, dtype=np.int64),
            '/nav_goal raw xyz',
        )

    if samples.goal_pose_points:
        return (
            _transform_points_to_odom(
                samples.goal_pose_times,
                samples.goal_pose_points,
                samples.goal_pose_frames,
                odom_times,
                odom_positions,
                odom_quaternions,
            ),
            np.asarray(samples.goal_pose_times, dtype=np.int64),
            '/goal_pose',
        )

    return np.empty((0, 3), dtype=float), np.empty((0,), dtype=np.int64), 'nav goal'


def _plot_xyz_series(
    ax,
    times_sec: np.ndarray,
    points: np.ndarray,
    label_prefix: str,
    linestyle: str,
    alpha: float = 0.95,
) -> None:
    if len(times_sec) == 0 or len(points) == 0:
        return

    axis_styles = (
        ('x', 'tab:red'),
        ('y', 'tab:green'),
        ('z', 'tab:blue'),
    )
    for axis_idx, (axis_name, color) in enumerate(axis_styles):
        ax.plot(
            times_sec,
            points[:, axis_idx],
            color=color,
            linestyle=linestyle,
            linewidth=1.15,
            alpha=alpha,
            label=f'{label_prefix} {axis_name}',
        )


def _draw_vector(
    ax,
    start: Sequence[float],
    end: Sequence[float],
    color: str,
    label: str,
    linewidth: float = 2.2,
) -> None:
    start_arr = np.asarray(start, dtype=float)
    delta = np.asarray(end, dtype=float) - start_arr
    if np.linalg.norm(delta) <= 1e-9:
        return

    ax.quiver(
        start_arr[0],
        start_arr[1],
        start_arr[2],
        delta[0],
        delta[1],
        delta[2],
        color=color,
        linewidth=linewidth,
        arrow_length_ratio=0.22,
        label=label,
    )


def plot_navigation_before_pose_commands(
    bag_path: Path,
    samples: BagSamples,
    save_dir: Optional[Path],
    show: bool,
    elev: float,
    azim: float,
) -> None:
    if not samples.odom_times:
        print(f'[skip] {bag_path}: no odom samples after reading')
        return

    odom_times, odom_positions, odom_quaternions = _sort_odom(samples)
    window_end = samples.pose_start_time or int(odom_times[-1])
    window_start = int(odom_times[0])

    odom_mask = odom_times <= window_end
    if not np.any(odom_mask):
        odom_mask[0] = True
    odom_nav = odom_positions[odom_mask]

    object_centers_odom, _ = _transform_base_points_to_odom(
        samples.object_times,
        samples.object_centers_base,
        odom_times,
        odom_positions,
        odom_quaternions,
    )
    object_contacts_odom, _ = _transform_base_points_to_odom(
        samples.object_times,
        samples.object_contact_points_base,
        odom_times,
        odom_positions,
        odom_quaternions,
    )
    object_times = np.asarray(samples.object_times, dtype=np.int64)
    object_mask = _time_window_mask(object_times, end_ns=window_end)

    nav_goal_points, nav_goal_times, nav_goal_label = _nav_goal_points_odom(
        samples,
        odom_times,
        odom_positions,
        odom_quaternions,
    )
    nav_goal_mask = _time_window_mask(nav_goal_times, end_ns=window_end)

    nav_goal_arrow_start_points = _transform_points_to_odom(
        samples.nav_goal_arrow_times,
        samples.nav_goal_arrow_starts,
        samples.nav_goal_arrow_frames,
        odom_times,
        odom_positions,
        odom_quaternions,
    )
    nav_goal_arrow_end_points = _transform_points_to_odom(
        samples.nav_goal_arrow_times,
        samples.nav_goal_arrow_ends,
        samples.nav_goal_arrow_frames,
        odom_times,
        odom_positions,
        odom_quaternions,
    )
    nav_goal_arrow_times = np.asarray(samples.nav_goal_arrow_times, dtype=np.int64)
    nav_goal_arrow_mask = _time_window_mask(nav_goal_arrow_times, end_ns=window_end)

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    duration_sec = max((window_end - window_start) * 1e-9, 0.0)
    ax.set_title(
        f'{bag_path.name}: navigation before /pose_command '
        f'({duration_sec:.1f}s)'
    )
    ax.set_xlabel('odom x [m]')
    ax.set_ylabel('odom y [m]')
    ax.set_zlabel('odom z [m]')
    ax.view_init(elev=elev, azim=azim)

    ax.plot(
        odom_nav[:, 0],
        odom_nav[:, 1],
        odom_nav[:, 2],
        color='black',
        linewidth=1.6,
        label='robot /odom before push',
    )
    ax.scatter(*odom_nav[0], color='limegreen', s=55, label='bag start')
    ax.scatter(
        *odom_nav[-1],
        color='crimson',
        s=55,
        label='/pose_command start',
    )
    robot_heading_idx = int(np.nonzero(odom_mask)[0][-1])
    robot_heading_start = odom_positions[robot_heading_idx]
    robot_heading_end = robot_heading_start + (
        _quaternion_to_matrix(odom_quaternions[robot_heading_idx])
        @ np.array([0.25, 0.0, 0.0], dtype=float)
    )
    _draw_vector(
        ax,
        robot_heading_start,
        robot_heading_end,
        color='crimson',
        label='last robot heading',
    )

    if len(nav_goal_points) and np.any(nav_goal_mask):
        goal_points = nav_goal_points[nav_goal_mask]
        ax.plot(
            goal_points[:, 0],
            goal_points[:, 1],
            goal_points[:, 2],
            color='tab:cyan',
            linewidth=1.0,
            linestyle='--',
            alpha=0.75,
            label=nav_goal_label,
        )
        ax.scatter(
            goal_points[:, 0],
            goal_points[:, 1],
            goal_points[:, 2],
            color='tab:cyan',
            s=18,
            alpha=0.35,
        )
        ax.scatter(
            *goal_points[-1],
            color='deepskyblue',
            marker='*',
            s=115,
            label='latest nav goal',
        )

    if len(nav_goal_arrow_start_points) and np.any(nav_goal_arrow_mask):
        arrow_starts = nav_goal_arrow_start_points[nav_goal_arrow_mask]
        arrow_ends = nav_goal_arrow_end_points[nav_goal_arrow_mask]
        _draw_vector(
            ax,
            arrow_starts[-1],
            arrow_ends[-1],
            color='tab:cyan',
            label='last nav goal heading',
        )

    if len(object_centers_odom) and np.any(object_mask):
        object_points = object_centers_odom[object_mask]
        object_contact_points = object_contacts_odom[object_mask]
        object_times_window = object_times[object_mask]
        colors = _normalized_times(object_times_window, window_start, window_end)
        object_scatter = ax.scatter(
            object_points[:, 0],
            object_points[:, 1],
            object_points[:, 2],
            c=colors,
            cmap='autumn',
            s=18,
            alpha=0.75,
            label='object center',
        )
        ax.plot(
            object_points[:, 0],
            object_points[:, 1],
            object_points[:, 2],
            color='tab:orange',
            linewidth=1.0,
            alpha=0.6,
        )
        ax.scatter(
            object_contact_points[:, 0],
            object_contact_points[:, 1],
            object_contact_points[:, 2],
            color='tab:blue',
            s=8,
            alpha=0.35,
            label='closest object surface',
        )
        fig.colorbar(
            object_scatter,
            ax=ax,
            shrink=0.65,
            pad=0.08,
            label='object time before /pose_command',
        )

    ax.legend(loc='upper right')
    _set_axes_equal(ax)
    fig.tight_layout()

    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)
        output_path = (
            save_dir / f'{bag_path.name}_navigation_before_pose_command.png'
        )
        fig.savefig(output_path, dpi=180)
        print(f'[saved] {output_path}')

    if show:
        print(f'[show] {bag_path}; close navigation figure to continue')
        plt.show()
    else:
        plt.close(fig)


def plot_end_effector_object_force(
    bag_path: Path,
    samples: BagSamples,
    save_dir: Optional[Path],
    show: bool,
    pose_window_padding_sec: float,
    include_force: bool,
    elev: float,
    azim: float,
) -> None:
    if samples.pose_start_time is None or samples.pose_end_time is None:
        print(f'[skip] {bag_path}: no /pose_command samples for push plot')
        return

    pad_ns = int(max(float(pose_window_padding_sec), 0.0) * 1e9)
    window_start = int(samples.pose_start_time) - pad_ns
    window_end = int(samples.pose_end_time) + pad_ns
    pose_zero = int(samples.pose_start_time)

    pose_points_base = np.asarray(samples.pose_points_base, dtype=float)
    pose_times = np.asarray(samples.pose_times, dtype=np.int64)
    pose_mask = _time_window_mask(pose_times, start_ns=window_start, end_ns=window_end)

    object_centers_base = np.asarray(samples.object_centers_base, dtype=float)
    object_contacts_base = np.asarray(samples.object_contact_points_base, dtype=float)
    object_times = np.asarray(samples.object_times, dtype=np.int64)
    object_mask = _time_window_mask(
        object_times,
        start_ns=window_start,
        end_ns=window_end,
    )

    fig = plt.figure(figsize=(13, 11))
    grid = GridSpec(3, 1, figure=fig, height_ratios=[2.3, 1.15, 1.15])
    ax_3d = fig.add_subplot(grid[0], projection='3d')
    ax_position = fig.add_subplot(grid[1])
    ax_force = fig.add_subplot(grid[2], sharex=ax_position)

    fig.suptitle(
        f'{bag_path.name}: end-effector push, object motion, and force in base_link'
    )
    ax_3d.set_xlabel('base_link x [m]')
    ax_3d.set_ylabel('base_link y [m]')
    ax_3d.set_zlabel('base_link z [m]')
    ax_3d.view_init(elev=elev, azim=azim)

    ax_3d.scatter(0.0, 0.0, 0.0, color='black', s=40, label='base_link origin')
    _draw_vector(
        ax_3d,
        (0.0, 0.0, 0.0),
        (0.25, 0.0, 0.0),
        color='black',
        label='base_link +x',
        linewidth=1.6,
    )

    if len(pose_points_base) and np.any(pose_mask):
        pose_window = pose_points_base[pose_mask]
        ax_3d.plot(
            pose_window[:, 0],
            pose_window[:, 1],
            pose_window[:, 2],
            color='tab:purple',
            linewidth=1.7,
            label='/pose_command end effector',
        )
        ax_3d.scatter(*pose_window[0], color='mediumpurple', s=45, label='ee start')
        ax_3d.scatter(*pose_window[-1], color='indigo', s=45, label='ee end')
        _plot_xyz_series(
            ax_position,
            _seconds_from_start(pose_times[pose_mask], pose_zero),
            pose_window,
            'ee',
            '-',
        )

    if len(object_centers_base) and np.any(object_mask):
        object_window = object_centers_base[object_mask]
        object_contact_window = object_contacts_base[object_mask]
        object_times_window = object_times[object_mask]
        object_colors = _normalized_times(
            object_times_window,
            int(samples.pose_start_time),
            int(samples.pose_end_time),
        )
        object_scatter = ax_3d.scatter(
            object_window[:, 0],
            object_window[:, 1],
            object_window[:, 2],
            c=object_colors,
            cmap='autumn',
            s=20,
            alpha=0.8,
            label='object center',
        )
        ax_3d.plot(
            object_window[:, 0],
            object_window[:, 1],
            object_window[:, 2],
            color='tab:orange',
            linewidth=1.2,
            alpha=0.7,
        )
        ax_3d.scatter(
            object_contact_window[:, 0],
            object_contact_window[:, 1],
            object_contact_window[:, 2],
            color='tab:blue',
            s=8,
            alpha=0.35,
            label='closest object surface',
        )
        fig.colorbar(
            object_scatter,
            ax=ax_3d,
            shrink=0.72,
            pad=0.08,
            label='object time through /pose_command window',
        )
        _plot_xyz_series(
            ax_position,
            _seconds_from_start(object_times_window, pose_zero),
            object_window,
            'object',
            '--',
            alpha=0.85,
        )

    ax_3d.legend(loc='upper right')
    _set_axes_equal(ax_3d)

    ax_position.axvline(0.0, color='tab:purple', linestyle=':', linewidth=1.0)
    ax_position.axvline(
        (int(samples.pose_end_time) - pose_zero) * 1e-9,
        color='tab:purple',
        linestyle=':',
        linewidth=1.0,
    )
    ax_position.set_ylabel('base_link position [m]')
    ax_position.grid(True, alpha=0.25)
    ax_position.legend(loc='upper right', ncol=3, fontsize='small')

    if include_force and samples.lowstate_times:
        lowstate_times = np.asarray(samples.lowstate_times, dtype=np.int64)
        lowstate_mask = _time_window_mask(
            lowstate_times,
            start_ns=window_start,
            end_ns=window_end,
        )
        if np.any(lowstate_mask):
            t_force = _seconds_from_start(lowstate_times[lowstate_mask], pose_zero)
            foot_forces = np.asarray(samples.foot_forces, dtype=float)[lowstate_mask]
            for leg_idx, leg_name in enumerate(LEG_NAMES):
                ax_force.plot(
                    t_force,
                    foot_forces[:, leg_idx],
                    linewidth=1.0,
                    label=f'{leg_name} force',
                )
        else:
            ax_force.text(
                0.02,
                0.75,
                'no /lowstate samples in window',
                transform=ax_force.transAxes,
            )
    elif include_force:
        ax_force.text(
            0.02,
            0.75,
            'no /lowstate samples',
            transform=ax_force.transAxes,
        )
    else:
        ax_force.text(
            0.02,
            0.75,
            'force plot disabled',
            transform=ax_force.transAxes,
        )

    ax_force.axvline(0.0, color='tab:purple', linestyle=':', linewidth=1.0)
    ax_force.axvline(
        (int(samples.pose_end_time) - pose_zero) * 1e-9,
        color='tab:purple',
        linestyle=':',
        linewidth=1.0,
    )
    ax_force.set_xlabel('seconds from first /pose_command')
    ax_force.set_ylabel('/lowstate foot_force')
    ax_force.grid(True, alpha=0.25)
    if ax_force.lines:
        ax_force.legend(loc='upper right', ncol=4, fontsize='small')

    fig.tight_layout()

    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)
        output_path = save_dir / f'{bag_path.name}_end_effector_object_force.png'
        fig.savefig(output_path, dpi=180)
        print(f'[saved] {output_path}')

    if show:
        print(f'[show] {bag_path}; close end-effector/object/force figure to continue')
        plt.show()
    else:
        plt.close(fig)


METRIC_COLUMNS = (
    ('bag', 'bag', 'str'),
    ('nav_min_goal_distance_m', 'nav closest m', '.3f'),
    ('nav_yaw_error_at_closest_deg', 'nav yaw err deg', '.1f'),
    ('nav_final_goal_distance_m', 'nav final m', '.3f'),
    ('nav_path_length_m', 'nav path m', '.3f'),
    ('touch_detected', 'touch', 'bool'),
    ('fr_max_force_above_ground_n', 'FR max force N', '.1f'),
    ('fr_peak_force_time_sec', 'peak force s', '.2f'),
    ('fr_touch_duration_sec', 'touch dur s', '.2f'),
    ('object_displacement_m', 'object odom disp m', '.3f'),
    ('ee_to_object_surface_min_m', 'ee-object min m', '.3f'),
    ('pose_command_reached', 'pose cmd reached', 'bool'),
    ('pose_checkpoint_segment_index', 'reach seg', 'str'),
    ('pose_checkpoint_0_min_error_m', 'cp0 err m', '.3f'),
    ('pose_checkpoint_1_min_error_m', 'cp1 err m', '.3f'),
    ('pose_checkpoint_2_min_error_m', 'cp2 err m', '.3f'),
    ('pose_checkpoint_3_min_error_m', 'cp3 err m', '.3f'),
    ('pose_checkpoint_worst_min_error_m', 'cp worst err m', '.3f'),
    ('environment_slope_deg', 'slope deg', '.1f'),
    ('initial_x_m', 'init x m', '.3f'),
    ('initial_y_m', 'init y m', '.3f'),
    ('initial_yaw_deg', 'init yaw deg', '.1f'),
)


def _path_length_xy(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)))


def _goal_yaw_samples_odom(
    samples: BagSamples,
    odom_times: np.ndarray,
    odom_quaternions: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, str]:
    if samples.nav_goal_arrow_starts:
        starts = _transform_points_to_odom(
            samples.nav_goal_arrow_times,
            samples.nav_goal_arrow_starts,
            samples.nav_goal_arrow_frames,
            odom_times,
            np.zeros((len(odom_times), 3), dtype=float),
            odom_quaternions,
        )
        ends = _transform_points_to_odom(
            samples.nav_goal_arrow_times,
            samples.nav_goal_arrow_ends,
            samples.nav_goal_arrow_frames,
            odom_times,
            np.zeros((len(odom_times), 3), dtype=float),
            odom_quaternions,
        )
        deltas = ends - starts
        valid = np.linalg.norm(deltas[:, :2], axis=1) > 1e-9
        return (
            np.asarray(samples.nav_goal_arrow_times, dtype=np.int64)[valid],
            np.arctan2(deltas[valid, 1], deltas[valid, 0]),
            '/nav_goal_debug_markers arrow',
        )

    if samples.nav_goal_vectors:
        return (
            np.asarray(samples.nav_goal_times, dtype=np.int64),
            np.asarray([goal[3] for goal in samples.nav_goal_vectors], dtype=float),
            '/nav_goal raw yaw',
        )

    if samples.goal_pose_quaternions:
        yaws = []
        for sample_time, quat, frame_id in zip(
            samples.goal_pose_times,
            samples.goal_pose_quaternions,
            samples.goal_pose_frames,
        ):
            heading = _quaternion_to_matrix(quat) @ np.array([1.0, 0.0, 0.0])
            if _same_frame(frame_id, 'base_link') or not frame_id:
                odom_idx = int(
                    _nearest_odom_indices(
                        np.asarray([sample_time], dtype=np.int64),
                        odom_times,
                    )[0]
                )
                heading = _quaternion_to_matrix(odom_quaternions[odom_idx]) @ heading
            yaws.append(math.atan2(float(heading[1]), float(heading[0])))
        return (
            np.asarray(samples.goal_pose_times, dtype=np.int64),
            np.asarray(yaws, dtype=float),
            '/goal_pose yaw',
        )

    return np.empty((0,), dtype=np.int64), np.empty((0,), dtype=float), 'goal yaw'


def _latest_goal_odom(
    samples: BagSamples,
    odom_times: np.ndarray,
    odom_positions: np.ndarray,
    odom_quaternions: np.ndarray,
    window_end: int,
) -> Optional[Dict[str, object]]:
    goal_points, goal_times, goal_label = _nav_goal_points_odom(
        samples,
        odom_times,
        odom_positions,
        odom_quaternions,
    )
    if len(goal_points) == 0:
        return None

    eligible = np.nonzero(goal_times <= int(window_end))[0]
    if len(eligible) == 0:
        return None

    goal_idx = int(eligible[-1])
    goal_time = int(goal_times[goal_idx])
    yaw_times, yaws, yaw_label = _goal_yaw_samples_odom(
        samples,
        odom_times,
        odom_quaternions,
    )
    goal_yaw = None
    if len(yaw_times):
        yaw_eligible = np.nonzero(yaw_times <= int(window_end))[0]
        if len(yaw_eligible):
            nearest_yaw_idx = int(
                yaw_eligible[
                    np.argmin(np.abs(yaw_times[yaw_eligible] - goal_time))
                ]
            )
            goal_yaw = float(yaws[nearest_yaw_idx])

    return {
        'time_ns': goal_time,
        'point': goal_points[goal_idx],
        'yaw': goal_yaw,
        'label': goal_label,
        'yaw_label': yaw_label,
    }


def _environment_slope_metrics(
    samples: BagSamples,
    odom_times: np.ndarray,
    odom_quaternions: np.ndarray,
    window_end: int,
) -> Dict[str, object]:
    if not samples.surface_normals:
        return {}

    normal_times = np.asarray(samples.surface_normal_times, dtype=np.int64)
    normal_mask = normal_times <= int(window_end)
    if not np.any(normal_mask):
        normal_mask = np.ones((len(normal_times),), dtype=bool)

    normals_odom = _transform_vectors_to_odom(
        samples.surface_normal_times,
        samples.surface_normals,
        samples.surface_normal_frames,
        odom_times,
        odom_quaternions,
    )[normal_mask]
    norms = np.linalg.norm(normals_odom, axis=1)
    valid = norms > 1e-9
    if not np.any(valid):
        return {}

    unit_normals = normals_odom[valid] / norms[valid, None]
    slope_rad = np.arctan2(
        np.linalg.norm(unit_normals[:, :2], axis=1),
        np.abs(unit_normals[:, 2]),
    )
    return {
        'environment_slope_deg': float(np.degrees(np.median(slope_rad))),
        'environment_slope_mean_deg': float(np.degrees(np.mean(slope_rad))),
        'surface_normal_samples': int(len(slope_rad)),
    }


def _aligned_pose_window(
    samples: BagSamples,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    if (
        samples.pose_start_time is None
        or samples.pose_end_time is None
        or not samples.lowstate_times
        or not samples.pose_times
    ):
        return None

    lowstate_times = np.asarray(samples.lowstate_times, dtype=np.int64)
    lowstate_mask = _time_window_mask(
        lowstate_times,
        start_ns=int(samples.pose_start_time),
        end_ns=int(samples.pose_end_time),
    )
    if not np.any(lowstate_mask):
        return None

    window_times = lowstate_times[lowstate_mask]
    pose_times = np.asarray(samples.pose_times, dtype=np.int64)
    pose_indices = _nearest_odom_indices(window_times, pose_times)
    pose_points = np.asarray(samples.pose_points_base, dtype=float)[pose_indices]
    foot_forces = np.asarray(samples.foot_forces, dtype=float)[lowstate_mask]
    joint_positions = np.asarray(samples.joint_positions, dtype=float)[lowstate_mask]
    return window_times, pose_indices, pose_points, foot_forces, joint_positions


def _pose_command_active_intervals(
    samples: BagSamples,
) -> List[Tuple[int, int]]:
    if not samples.pose_times or not samples.pose_points_base:
        return []

    pose_times = np.asarray(samples.pose_times, dtype=np.int64)
    pose_points = np.asarray(samples.pose_points_base, dtype=float)
    distances = np.linalg.norm(pose_points - pose_points[0], axis=1)
    active_indices = np.nonzero(distances > 1e-4)[0]
    if len(active_indices) == 0:
        return [(int(pose_times[0]), int(pose_times[-1]))]

    split_points = np.nonzero(np.diff(active_indices) > 1)[0] + 1
    intervals = []
    for active_segment in np.split(active_indices, split_points):
        start_idx = max(int(active_segment[0]) - 1, 0)
        end_idx = min(int(active_segment[-1]) + 1, len(pose_times) - 1)
        intervals.append((int(pose_times[start_idx]), int(pose_times[end_idx])))
    return intervals


def _select_reach_interval_by_force(
    samples: BagSamples,
    window_times: np.ndarray,
    pose_points: np.ndarray,
    foot_forces: np.ndarray,
    config: MetricsConfig,
) -> Optional[Dict[str, object]]:
    intervals = _pose_command_active_intervals(samples)
    if not intervals:
        return None

    best: Optional[Dict[str, object]] = None
    best_score = -math.inf
    for interval_idx, (start_ns, end_ns) in enumerate(intervals):
        window_indices = np.nonzero(
            _time_window_mask(window_times, start_ns=start_ns, end_ns=end_ns)
        )[0]
        if len(window_indices) == 0:
            score = -math.inf
            analysis_indices = window_indices
        else:
            above_ground = (
                pose_points[window_indices, 2] > float(config.ee_ground_clearance)
            )
            analysis_indices = (
                window_indices[above_ground] if np.any(above_ground) else window_indices
            )
            score = float(np.max(foot_forces[analysis_indices, 0]))

        if best is None or score > best_score:
            best_score = score
            best = {
                'index': int(interval_idx),
                'count': int(len(intervals)),
                'start_ns': int(start_ns),
                'end_ns': int(end_ns),
                'window_indices': window_indices,
                'analysis_indices': analysis_indices,
                'max_force_n': score if math.isfinite(score) else None,
            }

    return best


def _pose_checkpoint_targets(
    samples: BagSamples,
    active_start_ns: int,
    active_end_ns: int,
) -> Tuple[Dict[str, np.ndarray], str]:
    if not samples.pose_times or not samples.pose_points_base:
        return {}, 'none'

    pose_times = np.asarray(samples.pose_times, dtype=np.int64)
    pose_points = np.asarray(samples.pose_points_base, dtype=float)
    window_mask = _time_window_mask(
        pose_times,
        start_ns=active_start_ns,
        end_ns=active_end_ns,
    )

    pose_times_window = pose_times[window_mask]
    pose_window = pose_points[window_mask]
    if len(pose_window) == 0:
        pose_times_window = pose_times
        pose_window = pose_points

    checkpoint_times = np.linspace(
        int(pose_times_window[0]),
        int(pose_times_window[-1]),
        num=4,
    ).astype(np.int64)
    checkpoint_indices = _nearest_odom_indices(checkpoint_times, pose_times_window)
    return {
        f'{idx}': pose_window[int(point_idx)]
        for idx, point_idx in enumerate(checkpoint_indices)
    }, 'pose_command_equal_time_intervals'


def _median_sample_period_sec(times_ns: np.ndarray) -> float:
    if len(times_ns) < 2:
        return 0.0
    return float(np.median(np.diff(times_ns).astype(float)) * 1e-9)


def _force_touch_metrics(
    samples: BagSamples,
    config: MetricsConfig,
) -> Dict[str, object]:
    aligned = _aligned_pose_window(samples)
    if aligned is None or samples.pose_start_time is None:
        return {'touch_detected': False}

    window_times, _pose_indices, pose_points, foot_forces, _joint_positions = aligned
    reach_interval = _select_reach_interval_by_force(
        samples,
        window_times,
        pose_points,
        foot_forces,
        config,
    )
    if reach_interval is None:
        return {
            'touch_detected': False,
            'force_samples_above_ground': 0,
        }

    analysis_indices = np.asarray(reach_interval['analysis_indices'], dtype=int)
    if len(analysis_indices) == 0:
        return {
            'touch_detected': False,
            'pose_checkpoint_segment_count': int(reach_interval['count']),
            'pose_checkpoint_segment_index': int(reach_interval['index']),
            'force_samples_above_ground': 0,
        }

    fr_forces = foot_forces[analysis_indices, 0]
    peak_local_idx = int(np.argmax(fr_forces))
    peak_idx = int(analysis_indices[peak_local_idx])
    peak_force = float(fr_forces[peak_local_idx])
    over_threshold = fr_forces >= float(config.touch_force_threshold)
    rel_times = (
        window_times[analysis_indices].astype(float) - float(samples.pose_start_time)
    ) * 1e-9
    excess_force = np.maximum(
        fr_forces - float(config.touch_force_threshold),
        0.0,
    )
    impulse = float(np.trapz(excess_force, rel_times)) if len(rel_times) > 1 else 0.0
    return {
        'touch_detected': bool(peak_force >= float(config.touch_force_threshold)),
        'fr_max_force_above_ground_n': peak_force,
        'fr_peak_force_time_sec': (
            int(window_times[peak_idx]) - int(samples.pose_start_time)
        )
        * 1e-9,
        'fr_touch_duration_sec': float(
            np.count_nonzero(over_threshold)
            * _median_sample_period_sec(window_times[analysis_indices])
        ),
        'fr_force_impulse_over_threshold_n_s': impulse,
        'force_samples_above_ground': int(len(fr_forces)),
        'pose_checkpoint_segment_count': int(reach_interval['count']),
        'pose_checkpoint_segment_index': int(reach_interval['index']),
    }


def _pose_command_tracking_metrics(
    samples: BagSamples,
    config: MetricsConfig,
) -> Dict[str, object]:
    aligned = _aligned_pose_window(samples)
    if aligned is None or samples.pose_start_time is None:
        return {'pose_command_reached': False}

    window_times, _pose_indices, pose_points, foot_forces, joint_positions = aligned
    if len(joint_positions) == 0:
        return {'pose_command_reached': False}

    actual_fr = np.asarray(
        [_fr_foot_position_from_joints(joints) for joints in joint_positions],
        dtype=float,
    )
    calibration_end = int(samples.pose_start_time) + int(
        max(float(config.pose_command_calibration_sec), 0.0) * 1e9
    )
    calibration_mask = (window_times <= calibration_end) | (
        pose_points[:, 2] <= 0.5 * float(config.ee_ground_clearance)
    )
    if not np.any(calibration_mask):
        calibration_mask = np.zeros((len(window_times),), dtype=bool)
        calibration_mask[: min(len(window_times), 10)] = True

    command_offset = np.median(
        actual_fr[calibration_mask] - pose_points[calibration_mask],
        axis=0,
    )
    aligned_fr = actual_fr - command_offset

    reach_interval = _select_reach_interval_by_force(
        samples,
        window_times,
        pose_points,
        foot_forces,
        config,
    )
    if reach_interval is None:
        return {'pose_command_reached': False}
    active_start_ns = int(reach_interval['start_ns'])
    active_end_ns = int(reach_interval['end_ns'])
    window_indices = np.asarray(reach_interval['window_indices'], dtype=int)
    actual_trace = aligned_fr[window_indices]
    if len(actual_trace) == 0:
        return {'pose_command_reached': False}

    checkpoint_targets, checkpoint_source = _pose_checkpoint_targets(
        samples,
        active_start_ns,
        active_end_ns,
    )
    if not checkpoint_targets:
        return {'pose_command_reached': False}
    checkpoint_errors = {
        label: float(np.min(np.linalg.norm(actual_trace - target, axis=1)))
        for label, target in checkpoint_targets.items()
    }
    all_errors = np.asarray(list(checkpoint_errors.values()), dtype=float)
    if len(all_errors) == 0:
        return {'pose_command_reached': False}

    metrics: Dict[str, object] = {
        'pose_command_reached': bool(
            float(np.max(all_errors)) <= float(config.pose_reach_tolerance)
        ),
        'pose_checkpoint_source': checkpoint_source,
        'pose_checkpoint_count': int(len(checkpoint_targets)),
        'pose_checkpoint_segment_count': int(reach_interval['count']),
        'pose_checkpoint_segment_index': int(reach_interval['index']),
        'pose_checkpoint_segment_max_force_n': reach_interval['max_force_n'],
        'pose_checkpoint_active_start_sec': (
            int(active_start_ns) - int(samples.pose_start_time)
        )
        * 1e-9,
        'pose_checkpoint_active_duration_sec': (
            int(active_end_ns) - int(active_start_ns)
        )
        * 1e-9,
        'pose_checkpoint_best_min_error_m': float(np.min(all_errors)),
        'pose_checkpoint_worst_min_error_m': float(np.max(all_errors)),
        'pose_checkpoint_mean_min_error_m': float(np.mean(all_errors)),
        # Backwards-compatible key for callers that already consume this field.
        'pose_cmd_min_error_m': float(np.min(all_errors)),
    }
    for label, target in checkpoint_targets.items():
        metrics[f'pose_checkpoint_{label}_x_m'] = float(target[0])
        metrics[f'pose_checkpoint_{label}_y_m'] = float(target[1])
        metrics[f'pose_checkpoint_{label}_z_m'] = float(target[2])
        metrics[f'pose_checkpoint_{label}_min_error_m'] = checkpoint_errors[label]

    return metrics


def _object_motion_metrics(
    samples: BagSamples,
    config: MetricsConfig,
    odom_times: np.ndarray,
    odom_positions: np.ndarray,
    odom_quaternions: np.ndarray,
) -> Dict[str, object]:
    if samples.pose_start_time is None or samples.pose_end_time is None:
        return {}

    metrics: Dict[str, object] = {}
    reach_start_ns = int(samples.pose_start_time)
    reach_end_ns = int(samples.pose_end_time)
    aligned = _aligned_pose_window(samples)
    if aligned is not None:
        window_times, _pose_indices, pose_points, foot_forces, _joint_positions = aligned
        reach_interval = _select_reach_interval_by_force(
            samples,
            window_times,
            pose_points,
            foot_forces,
            config,
        )
        if reach_interval is not None:
            reach_start_ns = int(reach_interval['start_ns'])
            reach_end_ns = int(reach_interval['end_ns'])
            metrics['pose_checkpoint_segment_count'] = int(reach_interval['count'])
            metrics['pose_checkpoint_segment_index'] = int(reach_interval['index'])

    object_times = np.asarray(samples.object_times, dtype=np.int64)
    if len(object_times):
        object_centers_odom, _ = _transform_base_points_to_odom(
            samples.object_times,
            samples.object_centers_base,
            odom_times,
            odom_positions,
            odom_quaternions,
        )
        object_order = np.argsort(object_times)
        object_times_sorted = object_times[object_order]
        object_centers_sorted = object_centers_odom[object_order]
        start_idx, end_idx = _nearest_odom_indices(
            np.asarray([reach_start_ns, reach_end_ns], dtype=np.int64),
            object_times_sorted,
        )
        start_point = object_centers_sorted[int(start_idx)]
        end_point = object_centers_sorted[int(end_idx)]
        metrics['object_displacement_m'] = float(
            np.linalg.norm(end_point[:2] - start_point[:2])
        )
        metrics['object_displacement_start_time_sec'] = float(
            (int(object_times_sorted[int(start_idx)]) - int(samples.pose_start_time))
            * 1e-9
        )
        metrics['object_displacement_end_time_sec'] = float(
            (int(object_times_sorted[int(end_idx)]) - int(samples.pose_start_time))
            * 1e-9
        )

        object_mask = _time_window_mask(
            object_times_sorted,
            start_ns=reach_start_ns,
            end_ns=reach_end_ns,
        )
        if np.any(object_mask):
            object_points_window = object_centers_sorted[object_mask]
            object_delta = object_points_window - start_point
            metrics['object_max_displacement_m'] = float(
                max(
                    np.linalg.norm(end_point[:2] - start_point[:2]),
                    np.max(np.linalg.norm(object_delta[:, :2], axis=1)),
                )
            )

    if samples.pose_times and samples.object_contact_points_base:
        contact_times = np.asarray(samples.object_times, dtype=np.int64)
        contact_mask = _time_window_mask(
            contact_times,
            start_ns=reach_start_ns,
            end_ns=reach_end_ns,
        )
        if np.any(contact_mask):
            contact_points = np.asarray(
                samples.object_contact_points_base,
                dtype=float,
            )[contact_mask]
            pose_times = np.asarray(samples.pose_times, dtype=np.int64)
            contact_pose_indices = _nearest_odom_indices(
                contact_times[contact_mask],
                pose_times,
            )
            pose_points = np.asarray(samples.pose_points_base, dtype=float)[
                contact_pose_indices
            ]
            above_ground = pose_points[:, 2] > float(config.ee_ground_clearance)
            if np.any(above_ground):
                distances = np.linalg.norm(
                    pose_points[above_ground] - contact_points[above_ground],
                    axis=1,
                )
                metrics['ee_to_object_surface_min_m'] = float(np.min(distances))

    return metrics


def compute_experiment_metrics(
    bag_path: Path,
    samples: BagSamples,
    config: MetricsConfig,
) -> Dict[str, object]:
    metrics: Dict[str, object] = {'bag': bag_path.name}
    if not samples.odom_times:
        return metrics

    odom_times, odom_positions, odom_quaternions = _sort_odom(samples)
    window_start = int(odom_times[0])
    window_end = samples.pose_start_time or int(odom_times[-1])
    nav_mask = odom_times <= int(window_end)
    if not np.any(nav_mask):
        nav_mask[0] = True

    initial_roll, initial_pitch, initial_yaw = _quaternion_to_rpy(odom_quaternions[0])
    metrics.update(
        {
            'initial_x_m': float(odom_positions[0, 0]),
            'initial_y_m': float(odom_positions[0, 1]),
            'initial_z_m': float(odom_positions[0, 2]),
            'initial_roll_deg': float(math.degrees(initial_roll)),
            'initial_pitch_deg': float(math.degrees(initial_pitch)),
            'initial_yaw_deg': float(math.degrees(initial_yaw)),
            'navigation_duration_sec': float((int(window_end) - window_start) * 1e-9),
            'nav_path_length_m': _path_length_xy(odom_positions[nav_mask]),
        }
    )

    goal = _latest_goal_odom(
        samples,
        odom_times,
        odom_positions,
        odom_quaternions,
        int(window_end),
    )
    if goal is not None:
        goal_point = np.asarray(goal['point'], dtype=float)
        nav_indices = np.nonzero(nav_mask)[0]
        nav_positions = odom_positions[nav_indices]
        distances_xy = np.linalg.norm(nav_positions[:, :2] - goal_point[:2], axis=1)
        closest_local_idx = int(np.argmin(distances_xy))
        closest_odom_idx = int(nav_indices[closest_local_idx])
        final_odom_idx = int(nav_indices[-1])
        metrics.update(
            {
                'goal_x_m': float(goal_point[0]),
                'goal_y_m': float(goal_point[1]),
                'goal_z_m': float(goal_point[2]),
                'goal_source': str(goal['label']),
                'nav_min_goal_distance_m': float(distances_xy[closest_local_idx]),
                'nav_min_goal_distance_3d_m': float(
                    np.linalg.norm(odom_positions[closest_odom_idx] - goal_point)
                ),
                'nav_final_goal_distance_m': float(
                    np.linalg.norm(odom_positions[final_odom_idx, :2] - goal_point[:2])
                ),
                'nav_closest_time_sec': float(
                    (int(odom_times[closest_odom_idx]) - window_start) * 1e-9
                ),
            }
        )
        if goal['yaw'] is not None:
            goal_yaw = float(goal['yaw'])
            robot_yaw = _yaw_from_quaternion(odom_quaternions[closest_odom_idx])
            final_yaw = _yaw_from_quaternion(odom_quaternions[final_odom_idx])
            metrics.update(
                {
                    'goal_yaw_deg': float(math.degrees(goal_yaw)),
                    'nav_yaw_error_at_closest_deg': float(
                        abs(math.degrees(_angle_diff_rad(robot_yaw, goal_yaw)))
                    ),
                    'nav_final_yaw_error_deg': float(
                        abs(math.degrees(_angle_diff_rad(final_yaw, goal_yaw)))
                    ),
                }
            )

    metrics.update(
        _environment_slope_metrics(
            samples,
            odom_times,
            odom_quaternions,
            int(window_end),
        )
    )
    metrics.update(_force_touch_metrics(samples, config))
    metrics.update(
        _object_motion_metrics(
            samples,
            config,
            odom_times,
            odom_positions,
            odom_quaternions,
        )
    )
    metrics.update(_pose_command_tracking_metrics(samples, config))
    return metrics


def _format_metric(value: object, fmt: str) -> str:
    if value is None:
        return 'n/a'
    if fmt == 'bool':
        return 'yes' if bool(value) else 'no'
    if fmt == 'str':
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return 'n/a'
        return format(value, fmt)
    if isinstance(value, (np.floating,)):
        value_float = float(value)
        if not math.isfinite(value_float):
            return 'n/a'
        return format(value_float, fmt)
    return str(value)


def metrics_markdown(metrics_rows: Sequence[Dict[str, object]]) -> str:
    if not metrics_rows:
        return ''

    headers = [label for _key, label, _fmt in METRIC_COLUMNS]
    rows = [
        [
            _format_metric(row.get(key), fmt)
            for key, _label, fmt in METRIC_COLUMNS
        ]
        for row in metrics_rows
    ]
    widths = [
        max(len(header), *(len(row[idx]) for row in rows))
        for idx, header in enumerate(headers)
    ]
    header_line = '| ' + ' | '.join(
        header.ljust(widths[idx]) for idx, header in enumerate(headers)
    ) + ' |'
    separator_line = '| ' + ' | '.join('-' * width for width in widths) + ' |'
    row_lines = [
        '| '
        + ' | '.join(value.ljust(widths[idx]) for idx, value in enumerate(row))
        + ' |'
        for row in rows
    ]
    return '\n'.join([header_line, separator_line, *row_lines])


def write_metrics_csv(path: Path, metrics_rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [key for key, _label, _fmt in METRIC_COLUMNS]
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in metrics_rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def write_metrics_markdown(
    path: Path,
    metrics_rows: Sequence[Dict[str, object]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(metrics_markdown(metrics_rows) + '\n')


def _resolve_candidates(args) -> List[Path]:
    if args.bag:
        return [Path(bag).expanduser().resolve() for bag in args.bag]
    return find_dated_bags(args.bag_root, args.date, args.recursive)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Read valid ROS 2 bags one by one and create split navigation and '
            'end-effector/object/force matplotlib plots.'
        )
    )
    parser.add_argument(
        '--bag',
        action='append',
        help='Explicit rosbag2 directory. Can be passed more than once.',
    )
    parser.add_argument('--bag-root', default=str(Path.home()))
    parser.add_argument('--date', default=DEFAULT_DATE_SUBSTRING)
    parser.add_argument('--recursive', action='store_true')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--goal-pose-topic', default='/goal_pose')
    parser.add_argument('--nav-goal-topic', default='/nav_goal')
    parser.add_argument('--nav-goal-marker-topic', default='/nav_goal_debug_markers')
    parser.add_argument(
        '--obstacle-topic',
        default='/lidar_obstacle_detection/obstacle_list',
    )
    parser.add_argument('--pose-command-topic', default='/pose_command')
    parser.add_argument('--lowstate-topic', default='/lowstate')
    parser.add_argument(
        '--nav-goal-stride',
        type=int,
        default=1,
        help='Use one nav-goal sample for every N recorded nav-goal messages.',
    )
    parser.add_argument('--pose-stride', type=int, default=1)
    parser.add_argument('--obstacle-list-stride', type=int, default=1)
    parser.add_argument(
        '--lowstate-stride',
        type=int,
        default=1,
        help='Use one lowstate sample for every N recorded lowstate messages.',
    )
    parser.add_argument('--max-obstacle-lists', type=int, default=None)
    parser.add_argument('--save-dir', type=Path, default=None)
    parser.add_argument(
        '--no-show',
        action='store_true',
        help='Do not open matplotlib windows. Useful with --save-dir.',
    )
    parser.add_argument('--elev', type=float, default=28.0)
    parser.add_argument('--azim', type=float, default=-62.0)
    parser.add_argument(
        '--no-lowstate-plots',
        action='store_true',
        help='Skip force traces in the end-effector/object/force figure.',
    )
    parser.add_argument(
        '--pose-window-padding-sec',
        type=float,
        default=0.25,
        help='Seconds of context before and after /pose_command in the second figure.',
    )
    parser.add_argument(
        '--metrics-only',
        action='store_true',
        help='Print/save experiment metrics without creating plots.',
    )
    parser.add_argument(
        '--no-metrics',
        action='store_true',
        help='Skip the experiment metrics table.',
    )
    parser.add_argument(
        '--metrics-csv',
        type=Path,
        default=None,
        help='Write the experiment metrics table as CSV.',
    )
    parser.add_argument(
        '--metrics-md',
        type=Path,
        default=None,
        help='Write the experiment metrics table as Markdown.',
    )
    parser.add_argument(
        '--touch-force-threshold',
        type=float,
        default=25.0,
        help='FR foot_force threshold used to mark object touch while above ground.',
    )
    parser.add_argument(
        '--ee-ground-clearance',
        type=float,
        default=0.02,
        help='Minimum /pose_command z treated as end-effector above ground.',
    )
    parser.add_argument(
        '--pose-reach-tolerance',
        type=float,
        default=0.05,
        help='Best-effort FK/command error threshold for pose_command reached.',
    )
    parser.add_argument(
        '--pose-command-calibration-sec',
        type=float,
        default=1.0,
        help='Initial seconds used to align FR FK coordinates to /pose_command.',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    nav_goal_stride = max(int(args.nav_goal_stride), 1)
    pose_stride = max(int(args.pose_stride), 1)
    obstacle_list_stride = max(int(args.obstacle_list_stride), 1)
    lowstate_stride = max(int(args.lowstate_stride), 1)
    metrics_config = MetricsConfig(
        touch_force_threshold=float(args.touch_force_threshold),
        ee_ground_clearance=float(args.ee_ground_clearance),
        pose_reach_tolerance=float(args.pose_reach_tolerance),
        pose_command_calibration_sec=float(args.pose_command_calibration_sec),
    )

    candidates = _resolve_candidates(args)
    bags = _valid_bags(
        candidates,
        args.odom_topic,
        args.obstacle_topic,
        args.pose_command_topic,
    )

    if not bags:
        print('[done] No valid bags found.')
        return

    action = 'Processing' if args.metrics_only else 'Visualizing'
    print(f'[info] {action} {len(bags)} valid bag(s)')
    metrics_rows: List[Dict[str, object]] = []
    for bag_path in bags:
        print(f'[read] {bag_path}')
        samples = read_bag_samples(
            bag_path=bag_path,
            odom_topic=args.odom_topic,
            goal_pose_topic=args.goal_pose_topic,
            nav_goal_topic=args.nav_goal_topic,
            nav_goal_marker_topic=args.nav_goal_marker_topic,
            obstacle_topic=args.obstacle_topic,
            pose_command_topic=args.pose_command_topic,
            lowstate_topic=args.lowstate_topic,
            nav_goal_stride=nav_goal_stride,
            pose_stride=pose_stride,
            obstacle_list_stride=obstacle_list_stride,
            lowstate_stride=lowstate_stride,
            max_obstacle_lists=args.max_obstacle_lists,
        )
        print(
            '[info] samples: odom=%d nav_goals=%d nav_goal_markers=%d '
            'nav_goal_arrows=%d goal_poses=%d '
            'objects=%d obstacle_centers=%d normals=%d pose_commands=%d lowstate=%d'
            % (
                len(samples.odom_times),
                len(samples.nav_goal_times),
                len(samples.nav_goal_marker_times),
                len(samples.nav_goal_arrow_times),
                len(samples.goal_pose_times),
                len(samples.object_times),
                len(samples.obstacle_times),
                len(samples.surface_normals),
                len(samples.pose_times),
                len(samples.lowstate_times),
            )
        )
        if not args.no_metrics:
            metrics_rows.append(
                compute_experiment_metrics(
                    bag_path=bag_path,
                    samples=samples,
                    config=metrics_config,
                )
            )
        if not args.metrics_only:
            plot_navigation_before_pose_commands(
                bag_path=bag_path,
                samples=samples,
                save_dir=args.save_dir,
                show=not args.no_show,
                elev=args.elev,
                azim=args.azim,
            )
            plot_end_effector_object_force(
                bag_path=bag_path,
                samples=samples,
                save_dir=args.save_dir,
                show=not args.no_show,
                pose_window_padding_sec=args.pose_window_padding_sec,
                include_force=not args.no_lowstate_plots,
                elev=args.elev,
                azim=args.azim,
            )

    if metrics_rows:
        print('\n[metrics] Experiment summary')
        print(metrics_markdown(metrics_rows))
        metrics_csv = args.metrics_csv
        metrics_md = args.metrics_md
        if args.save_dir is not None:
            metrics_csv = metrics_csv or (args.save_dir / 'experiment_metrics.csv')
            metrics_md = metrics_md or (args.save_dir / 'experiment_metrics.md')
        if metrics_csv is not None:
            write_metrics_csv(metrics_csv.expanduser(), metrics_rows)
            print(f'[saved] {metrics_csv.expanduser()}')
        if metrics_md is not None:
            write_metrics_markdown(metrics_md.expanduser(), metrics_rows)
            print(f'[saved] {metrics_md.expanduser()}')


if __name__ == '__main__':
    main()
