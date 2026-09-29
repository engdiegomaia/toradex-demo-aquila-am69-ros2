# Gazebo Quadruped ros2_control Plugin

This package is a modified fork of
[`gz_ros2_control`](https://github.com/ros-controls/gz_ros2_control), vendored
from `legubiao/quadruped_ros2_control`. See [`PROVENANCE.md`](PROVENANCE.md)
for origin, license and edits.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to gz_quadruped_hardware --symlink-install
```
