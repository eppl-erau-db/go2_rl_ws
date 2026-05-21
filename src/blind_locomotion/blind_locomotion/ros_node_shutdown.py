"""Shared ROS node shutdown helpers."""

import rclpy
from rclpy.executors import ExternalShutdownException


def spin_until_shutdown(node):
    """Spin a node and suppress expected Ctrl-C shutdown tracebacks."""
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
