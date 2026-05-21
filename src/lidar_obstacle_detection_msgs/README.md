# `lidar_obstacle_detection_msgs`

ROS 2 **interface-only** package: message definitions used by **`lidar_obstacle_detection`** (node `lidar_cloud_ingress`) for obstacle output.

Full pipeline documentation, default topics, and the **algorithm parameter file** are in the **`lidar_obstacle_detection`** package:

- In a source checkout: `lidar_obstacle_detection/README.md` next to this folder.
- After install: `$(ros2 pkg prefix lidar_obstacle_detection)/share/lidar_obstacle_detection/README.md`

---

## Messages

| File | Description |
|------|-------------|
| `msg/Obstacle.msg` | One obstacle: AABB center `geometry_msgs/Point position`; extents `width`, `height`, `length` (m); `volume` (m³); `geometry_msgs/Vector3[] surface_normals` (typically one unit normal from ground under the XY footprint); `geometry_msgs/Point closest_surface_point` (YZ-binned robust minimum-X surface sample in the same frame as `position`; NaN components if invalid). |
| `msg/ObstacleList.msg` | `std_msgs/Header header` (use `output_cloud_frame_id`, e.g. `base_link`) + `Obstacle[] obstacles`. |

Dependencies: `geometry_msgs`, `std_msgs`.

Python pip dependencies for the **node** that uses these messages are listed in **`lidar_obstacle_detection/requirements.txt`** (not in this interface package).

---

## Build

```bash
colcon build --packages-select lidar_obstacle_detection_msgs
```

Python import example:

```python
from lidar_obstacle_detection_msgs.msg import Obstacle, ObstacleList
```
