#!/usr/bin/env python3
"""Convert recorded obstacle and goal topics into RViz MarkerArray messages."""

from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray

try:
    from lidar_obstacle_detection_msgs.msg import ObstacleList
except ImportError:
    ObstacleList = None


def _duration(seconds: float) -> Duration:
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))


def _is_finite_point(point) -> bool:
    return (
        math.isfinite(float(point.x))
        and math.isfinite(float(point.y))
        and math.isfinite(float(point.z))
    )


class BagMarkerNode(Node):
    """Publish RViz markers for obstacle boxes and nav goal poses."""

    def __init__(self) -> None:
        super().__init__('go2_bag_marker_node')

        self.declare_parameter(
            'obstacle_topic',
            '/lidar_obstacle_detection/obstacle_list',
        )
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('pose_command_topic', '/pose_command')
        self.declare_parameter('show_pose_command_markers', False)
        self.declare_parameter(
            'marker_topic',
            '/go2_bag_visualization/markers',
        )
        self.declare_parameter('fallback_frame_id', 'base_link')
        self.declare_parameter('pose_command_frame_id', 'base_link')
        self.declare_parameter('obstacle_lifetime_sec', 0.35)
        self.declare_parameter('goal_lifetime_sec', 0.0)
        self.declare_parameter('min_obstacle_scale', 0.03)
        self.declare_parameter('max_obstacle_markers', 100)
        self.declare_parameter('show_obstacle_labels', True)
        self.declare_parameter('show_closest_surface_points', True)

        self.obstacle_topic = self.get_parameter(
            'obstacle_topic'
        ).get_parameter_value().string_value
        self.goal_pose_topic = self.get_parameter(
            'goal_pose_topic'
        ).get_parameter_value().string_value
        self.pose_command_topic = self.get_parameter(
            'pose_command_topic'
        ).get_parameter_value().string_value
        self.show_pose_command_markers = bool(
            self.get_parameter('show_pose_command_markers').value
        )
        marker_topic = self.get_parameter(
            'marker_topic'
        ).get_parameter_value().string_value
        self.fallback_frame_id = self.get_parameter(
            'fallback_frame_id'
        ).get_parameter_value().string_value
        self.pose_command_frame_id = self.get_parameter(
            'pose_command_frame_id'
        ).get_parameter_value().string_value
        self.obstacle_lifetime = _duration(
            float(self.get_parameter('obstacle_lifetime_sec').value)
        )
        self.goal_lifetime = _duration(
            float(self.get_parameter('goal_lifetime_sec').value)
        )
        self.min_obstacle_scale = max(
            float(self.get_parameter('min_obstacle_scale').value),
            0.001,
        )
        self.max_obstacle_markers = max(
            int(self.get_parameter('max_obstacle_markers').value),
            1,
        )
        self.show_obstacle_labels = bool(
            self.get_parameter('show_obstacle_labels').value
        )
        self.show_closest_surface_points = bool(
            self.get_parameter('show_closest_surface_points').value
        )

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, marker_qos)
        self._last_obstacle_count = 0
        self._last_closest_ids = set()

        if ObstacleList is None:
            self.get_logger().error(
                'lidar_obstacle_detection_msgs is not importable; obstacle '
                'markers will be disabled until the workspace is built/sourced.'
            )
        else:
            self.create_subscription(
                ObstacleList,
                self.obstacle_topic,
                self._obstacle_callback,
                sensor_qos,
            )

        self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self._goal_pose_callback,
            sensor_qos,
        )
        if self.show_pose_command_markers:
            self.create_subscription(
                Pose,
                self.pose_command_topic,
                self._pose_command_callback,
                sensor_qos,
            )

        self.get_logger().info(
            'Publishing RViz markers on %s from obstacle=%s goal=%s'
            % (
                marker_topic,
                self.obstacle_topic,
                self.goal_pose_topic,
            )
        )
        if self.show_pose_command_markers:
            self.get_logger().info(
                'End-effector pose command markers enabled from %s'
                % self.pose_command_topic
            )

    def _frame_id(self, frame_id: str | None) -> str:
        return frame_id if frame_id else self.fallback_frame_id

    def _new_marker(self, frame_id: str, namespace: str, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id(frame_id)
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _delete_marker(self, namespace: str, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.fallback_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _obstacle_callback(self, msg) -> None:
        markers = []
        frame_id = self._frame_id(msg.header.frame_id)
        stamp = msg.header.stamp
        obstacle_count = min(len(msg.obstacles), self.max_obstacle_markers)
        closest_ids = set()

        for idx, obstacle in enumerate(msg.obstacles[:obstacle_count]):
            box = self._new_marker(frame_id, 'obstacles', idx)
            box.header.stamp = stamp
            box.type = Marker.CUBE
            box.lifetime = self.obstacle_lifetime
            box.pose.position = obstacle.position
            box.scale.x = max(float(obstacle.width), self.min_obstacle_scale)
            box.scale.y = max(float(obstacle.height), self.min_obstacle_scale)
            box.scale.z = max(float(obstacle.length), self.min_obstacle_scale)
            box.color.r = 1.0
            box.color.g = 0.36
            box.color.b = 0.06
            box.color.a = 0.55
            markers.append(box)

            if self.show_obstacle_labels:
                label = self._new_marker(frame_id, 'obstacle_labels', idx)
                label.header.stamp = stamp
                label.type = Marker.TEXT_VIEW_FACING
                label.lifetime = self.obstacle_lifetime
                label.pose.position.x = obstacle.position.x
                label.pose.position.y = obstacle.position.y
                label.pose.position.z = (
                    obstacle.position.z + max(float(obstacle.length), 0.0) * 0.5 + 0.08
                )
                label.scale.z = 0.11
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 0.9
                label.text = f'obs {idx}'
                markers.append(label)

            if (
                self.show_closest_surface_points
                and _is_finite_point(obstacle.closest_surface_point)
            ):
                closest = self._new_marker(frame_id, 'closest_surface_points', idx)
                closest.header.stamp = stamp
                closest.type = Marker.SPHERE
                closest.lifetime = self.obstacle_lifetime
                closest.pose.position = obstacle.closest_surface_point
                closest.scale.x = 0.06
                closest.scale.y = 0.06
                closest.scale.z = 0.06
                closest.color.r = 0.1
                closest.color.g = 0.65
                closest.color.b = 1.0
                closest.color.a = 0.95
                markers.append(closest)
                closest_ids.add(idx)

        for idx in range(obstacle_count, self._last_obstacle_count):
            markers.append(self._delete_marker('obstacles', idx))
            markers.append(self._delete_marker('obstacle_labels', idx))

        for idx in self._last_closest_ids - closest_ids:
            markers.append(self._delete_marker('closest_surface_points', idx))

        self._last_obstacle_count = obstacle_count
        self._last_closest_ids = closest_ids
        if markers:
            self.marker_pub.publish(MarkerArray(markers=markers))

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        frame_id = self._frame_id(msg.header.frame_id)
        stamp = msg.header.stamp
        markers = self._pose_markers(
            pose=msg.pose,
            frame_id=frame_id,
            stamp=stamp,
            namespace='goal_pose',
            label='goal',
            color=(0.1, 1.0, 0.25, 0.95),
        )
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _pose_command_callback(self, msg: Pose) -> None:
        markers = self._pose_markers(
            pose=msg,
            frame_id=self.pose_command_frame_id,
            stamp=self.get_clock().now().to_msg(),
            namespace='end_effector_pose_command',
            label='ee cmd',
            color=(0.85, 0.2, 1.0, 0.9),
        )
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _pose_markers(
        self,
        pose: Pose,
        frame_id: str,
        stamp,
        namespace: str,
        label: str,
        color,
    ):
        arrow = self._new_marker(frame_id, namespace, 0)
        arrow.header.stamp = stamp
        arrow.type = Marker.ARROW
        arrow.lifetime = self.goal_lifetime
        arrow.pose = pose
        arrow.scale.x = 0.35
        arrow.scale.y = 0.06
        arrow.scale.z = 0.06
        arrow.color.r = color[0]
        arrow.color.g = color[1]
        arrow.color.b = color[2]
        arrow.color.a = color[3]

        sphere = self._new_marker(frame_id, namespace, 1)
        sphere.header.stamp = stamp
        sphere.type = Marker.SPHERE
        sphere.lifetime = self.goal_lifetime
        sphere.pose.position = pose.position
        sphere.scale.x = 0.12
        sphere.scale.y = 0.12
        sphere.scale.z = 0.12
        sphere.color.r = color[0]
        sphere.color.g = color[1]
        sphere.color.b = color[2]
        sphere.color.a = min(color[3] + 0.05, 1.0)

        text = self._new_marker(frame_id, namespace, 2)
        text.header.stamp = stamp
        text.type = Marker.TEXT_VIEW_FACING
        text.lifetime = self.goal_lifetime
        text.pose.position.x = pose.position.x
        text.pose.position.y = pose.position.y
        text.pose.position.z = pose.position.z + 0.16
        text.scale.z = 0.11
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.9
        text.text = label

        return [arrow, sphere, text]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BagMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
