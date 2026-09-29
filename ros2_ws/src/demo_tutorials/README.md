# demo_tutorials

L1 learning code: publisher/subscriber, service, and launch-composition
examples used to onboard a first-time ROS 2 user on this project.

**Runs on:** either machine — plain rclpy nodes with no Gazebo, RViz, or
architecture-specific dependency.

## Overview

Not production code. Production nodes live in `demo_bringup`,
`demo_navigation`, `demo_perception`, `demo_simulation`, and `demo_description`.
This package exists so a newcomer can walk the full
publisher/subscriber/parameter/service/launch loop end to end without
touching demo-critical code.

## Nodes

| Node | Publishes | Subscribes | Services | Parameters |
| --- | --- | --- | --- | --- |
| `heartbeat_publisher` | `/demo/system/heartbeat` (`std_msgs/String`) | — | — | `rate_hz` (live-tunable) |
| `heartbeat_subscriber` | — | `/demo/system/heartbeat` | — | — |
| `add_two_ints_server` | — | — | offers `add_two_ints` (`example_interfaces/srv/AddTwoInts`), relative — run under the `/demo` namespace to get `/demo/add_two_ints` | — |

## Launch files

| File | What | Key arguments |
| --- | --- | --- |
| `launch/heartbeat.launch.py` | `heartbeat_publisher` + `heartbeat_subscriber` | `rate_hz` |
| `launch/turtlesim_demo.launch.py` | `turtlesim_node` + `turtle_teleop_key`, composed together | — |

## Build and test

```bash
cd ros2_ws
colcon build --symlink-install --packages-select demo_tutorials
colcon test --packages-select demo_tutorials && colcon test-result --verbose
```

## Usage

```bash
ros2 launch demo_tutorials heartbeat.launch.py rate_hz:=2.0
# Subscriber log should show ~2 Hz reception with no counter gaps.
```

Live parameter retuning:

```bash
ros2 param set /demo/heartbeat_publisher rate_hz 5.0
```

Service, started under the project namespace:

```bash
ros2 run demo_tutorials add_two_ints_server --ros-args -r __ns:=/demo
# separate terminal
ros2 service call /demo/add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"
```

## Notes

- This package intentionally has no dependency on `demo_bringup` or any other
  project package, so it can be built and run in isolation as a first
  exercise.
- `heartbeat_subscriber` logs a warning if it detects a gap in the received
  counter, which is the acceptance signal for the exit exercise above.
