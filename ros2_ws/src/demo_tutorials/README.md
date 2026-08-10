# demo_tutorials

**L1 learning code.** Not production. See `.ai/AGENTS.md` §9 (ML1) for the acceptance
criteria this package satisfies.

Production nodes live in `demo_bringup`, `demo_navigation`, `demo_perception`, and
friends. This package exists so a first-time ROS 2 user can experience the full
publisher / subscriber / parameter / service / launch loop end-to-end without
touching the demo-critical code.

## Contents

| Executable | Purpose |
| --- | --- |
| `heartbeat_publisher` | Publishes `std_msgs/String` on `/demo/system/heartbeat` at a `rate_hz` parameter |
| `heartbeat_subscriber` | Subscribes, logs receipt, detects gaps in the counter |
| `add_two_ints_server` | `example_interfaces/srv/AddTwoInts` server |

## Launch

- `heartbeat.launch.py` — publisher + subscriber with a `rate_hz` launch argument
- `turtlesim_demo.launch.py` — `turtlesim_node` + `turtle_teleop_key` composed together

## Exit exercise (L1 acceptance)

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch demo_tutorials heartbeat.launch.py rate_hz:=2.0
# Subscriber log should show ~2 Hz reception with no counter gaps.
```

Live parameter retuning:

```bash
ros2 param set /demo/heartbeat_publisher rate_hz 5.0
```

Service:

```bash
ros2 service call /demo/add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"
```
