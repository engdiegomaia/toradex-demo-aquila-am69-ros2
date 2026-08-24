# demo_perception

Synthetic-detection stub and its costmap adapter. Delivers the perception half
of ML3.

**Runs on:** Aquila AM69 (arm64) in `target` mode, x86 host in `learn` mode.
Identical code both ways — these nodes are CPU-only, with no GPU, OpenGL, or
NPU dependency yet.

This package is separated from day one even though it is a stub, because it
defines the **OTA update granularity**: swapping the stub for real TIDL/NPU
inference must be a container swap, never an interface change.

## The contract this package exists to protect

| Topic | Type | Direction |
| --- | --- | --- |
| `/demo/camera/image_raw` | `sensor_msgs/Image` | in |
| `/demo/perception/detections` | `vision_msgs/Detection2DArray` | out |
| `/demo/perception/detection_cloud` | `sensor_msgs/PointCloud2` | out (costmap adapter) |

Two invariants, from `CLAUDE.md` rule 6:

1. **The stub never learns where its images come from.** Gazebo, a USB camera,
   and a rosbag are indistinguishable from inside `detection_stub`.
2. **No consumer learns whether detections came from the stub or from real
   inference.** Nav2 and the HMI see `Detection2DArray` either way.

## Nodes

### `detection_stub`

Publishes one deterministic synthetic detection per received frame. The box
sweeps horizontally as a pure function of the received-frame counter, so a given
frame index always produces the same box — downstream behaviour stays
reproducible while the real model does not exist.

Parameters: `class_id`, `score`, `box_width_px`, `box_height_px`,
`period_frames`.

Subscribes with **reliable** QoS, matching the `ros_gz_bridge` camera publisher.
This is load-bearing for the 921600-byte raw images used by HIL: with a
best-effort reader, discovery succeeded but every fragmented sample was lost;
with a reliable reader, the module received the measured 10 Hz stream and can
request retransmission of a missing fragment.

### `detections_to_cloud`

Projects detections onto the ground plane and republishes them as `PointCloud2`
so Nav2's stock `ObstacleLayer` can consume them.

Parameters: `assumed_range_m`, `hfov_rad`, `image_width_px`,
`obstacle_height_m`, `points_per_detection`, `min_score`.

## Why an adapter instead of a C++ costmap plugin

`CLAUDE.md` requires detections to feed a Nav2 costmap layer, not just the HMI
screen. Nav2's stock `ObstacleLayer` reads `LaserScan` or `PointCloud2` — it
cannot read `Detection2DArray`. The two ways to bridge that are a custom C++
costmap plugin, or conversion to a message the stock layer already accepts.

Project convention is Python by default, C++ only where hardware-measured
performance justifies it. Nothing has been measured on the AM69 yet, so this is
the Python adapter. Replacing it with a native C++ layer later changes nothing
on either side: `demo_perception` still publishes `Detection2DArray`, and Nav2
still marks the same cells.

## Known limitation: there is no depth

A 2D bounding box carries no range information. `detections_to_cloud` assumes
every detection sits on the ground plane at a **fixed assumed distance**
(`assumed_range_m`, default 2.0 m) and uses the pinhole model to turn the box's
horizontal position into a bearing. The obstacle lands at roughly the right
angle, at an assumed range.

That is honest for a stub and sufficient to prove the costmap wiring. It is not
a substitute for depth. When real inference arrives, replace the assumed range
with a depth image, a stereo pair, or a detection carrying its own 3D pose.

Two consequences are deliberate and encoded in `demo_navigation/config/nav2_params.yaml`:

- `marking: true`, `clearing: false` — the projection is too coarse to be
  trusted for clearing. Letting it clear would allow a bad projection to erase
  real lidar obstacles.
- `observation_persistence: 1.0` — detections expire after a second rather than
  leaving a trail of phantom obstacles behind the robot.

## Bearing sign

Image `+x` runs right; ROS `+y` runs left. The adapter negates accordingly, and
`test_bearing_sign_follows_ros_convention` pins it. Getting this backwards
steers the robot *into* the obstacle it is avoiding, and nothing else in the
pipeline would catch it.

## Running

```bash
ros2 launch demo_perception perception.launch.py
```

Standalone, without a simulator, for inspection:

```bash
ros2 launch demo_perception perception.launch.py use_sim_time:=false
ros2 topic echo /demo/perception/detections
ros2 topic hz /demo/perception/detection_cloud
```

## Tests

```bash
colcon test --packages-select demo_perception && colcon test-result --verbose
```

16 unit tests cover determinism, frame bounds, header propagation, score
filtering, bearing sign, cloud structure, and the empty-detection case.

That last one matters more than it looks: the adapter must publish an **empty**
cloud when nothing is detected. Going silent is not the same as "no obstacles" —
the `ObstacleLayer` needs a continuous stream to age out old marks, so silence
freezes the last obstacle in the costmap and the robot refuses to plan through
it.
