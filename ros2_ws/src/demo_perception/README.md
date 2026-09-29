# demo_perception

Synthetic-detection stub, its Nav2 costmap adapter, and the maze-exit
fiducial detector.

**Runs on:** Aquila AM69 (arm64) in `hil` mode, x86 host in `learn` mode.
Identical code either way — every node here is CPU-only, with no GPU, OpenGL,
or NPU dependency.

## Overview

This package is separated from day one, even though `detection_stub` is a
stub, because it defines the **OTA update granularity**: swapping the stub for
real TIDL/NPU inference must be a container swap, never an interface change.
The contract it protects:

| Topic | Type | Direction |
| --- | --- | --- |
| `/demo/camera/image_raw` | `sensor_msgs/Image` | in |
| `/demo/perception/detections` | `vision_msgs/Detection2DArray` | out |
| `/demo/perception/detection_cloud` | `sensor_msgs/PointCloud2` | out (costmap adapter) |

Two invariants follow from the topic contract: the stub never learns where its
images come from (Gazebo, a USB camera, and a rosbag are indistinguishable
from inside it), and no consumer learns whether a detection came from the stub
or from real inference — Nav2 and the HMI see `Detection2DArray` either way.

## Nodes

| Node | Publishes | Subscribes | Notes |
| --- | --- | --- | --- |
| `detection_stub` | `/demo/perception/detections` (`Detection2DArray`) | `/demo/camera/image_raw` (reliable QoS) | One deterministic synthetic box per received frame; a pure function of the frame counter. |
| `detections_to_cloud` | `/demo/perception/detection_cloud` (`PointCloud2`) | `/demo/perception/detections` | Projects detections onto the ground plane at an assumed fixed range. |
| `maze_exit_detector` | `/demo/perception/maze_exit/{detections,pose,diagnostics}` | `/demo/camera/image_raw`, `/demo/camera/camera_info` | Fiducial (AprilTag) or magenta-blob detector for the maze exit marker; scenario-specific, not part of the topic contract above. |

| Parameter | Default | Node | Description |
| --- | --- | --- | --- |
| `class_id` | `'box'` | `detection_stub` | Reported class label. |
| `score` | `0.87` | `detection_stub` | Reported confidence. |
| `box_width_px` / `box_height_px` | `120.0` / `160.0` | `detection_stub` | Synthetic bounding-box size. |
| `period_frames` | `90.0` | `detection_stub` | Frames per sweep of the box across the image. |
| `assumed_range_m` | `2.0` | `detections_to_cloud` | Fixed ground-plane range assumed for every detection (no depth available). |
| `hfov_rad` | `1.089` | `detections_to_cloud` | Must match the camera's horizontal FOV in the URDF. |
| `image_width_px` | `640.0` | `detections_to_cloud` | Must match the camera's width in the URDF. |
| `obstacle_height_m` | `0.3` | `detections_to_cloud` | Height of the synthesized obstacle column. |
| `points_per_detection` | `5` | `detections_to_cloud` | Cloud density per detection. |
| `min_score` | `0.5` | `detections_to_cloud` | Detections below this confidence are dropped. |
| `detector_backend` | `'fiducial'` | `maze_exit_detector` | `fiducial` (AprilTag) or `magenta` (color blob). |
| `fiducial_dictionary` / `fiducial_id` | `'DICT_APRILTAG_36h11'` / `0` | `maze_exit_detector` | AprilTag dictionary and tag ID to track. |
| `fiducial_size_m` / `marker_width_m` | tag-specific / `0.8` | `maze_exit_detector` | Physical marker dimensions used for range estimation. |
| `confirm_frames` / `confirmation_window` | `3` / `5` | `maze_exit_detector` | Debounce: consecutive positive frames required within a sliding window before confirming a detection. |

## Launch files

| File | Starts | Key arguments |
| --- | --- | --- |
| `launch/perception.launch.py` | `detection_stub` + `detections_to_cloud` + `maze_exit_detector` | `use_sim_time`, `assumed_range_m`, `detector_backend` |

## Configuration

`config/` is currently empty (`.gitkeep` only) — every node above is
parameterized via launch arguments and ROS parameters rather than a static
file.

## Build and test

```bash
cd ros2_ws
colcon build --symlink-install --packages-select demo_perception
colcon test --packages-select demo_perception && colcon test-result --verbose
```

Standalone, without a simulator:

```bash
ros2 launch demo_perception perception.launch.py use_sim_time:=false
ros2 topic echo /demo/perception/detections
ros2 topic hz /demo/perception/detection_cloud
```

## Notes

- **Why an adapter instead of a C++ costmap plugin:** Nav2's stock
  `ObstacleLayer` reads `LaserScan` or `PointCloud2`, not `Detection2DArray`.
  Project convention is Python by default, C++ only where hardware-measured
  performance on the AM69 justifies it — nothing has been measured yet, so
  `detections_to_cloud` is the Python adapter. Replacing it with a native
  layer later changes nothing on either side of the boundary.
- **There is no depth.** A 2D bounding box carries no range information;
  `detections_to_cloud` assumes a fixed `assumed_range_m` and uses the
  pinhole model to convert horizontal box position into bearing. That is
  honest for a stub and sufficient to prove the costmap wiring, but it is not
  a substitute for real depth. `clearing: false` and
  `observation_persistence: 1.0` in `demo_navigation`'s costmap config exist
  specifically because this projection is too coarse to trust for clearing.
- **Bearing sign matters.** Image `+x` runs right; ROS `+y` runs left. The
  adapter negates accordingly, pinned by
  `test_bearing_sign_follows_ros_convention` — getting this backwards steers
  the robot into the obstacle it should avoid, silently.
- **An empty detection list must still publish an empty cloud.** Going silent
  is not the same as "no obstacles": `ObstacleLayer` needs a continuous
  stream to age out old marks, and silence freezes the last obstacle in the
  costmap.
- **`detection_stub` subscribes with reliable QoS**, matching the
  `ros_gz_bridge` camera publisher. This is load-bearing on the HIL Ethernet
  link for the raw, uncompressed image size used by the demo: a best-effort
  reader lost every fragmented sample.
