# demo_description

Robot model for the Aquila AM69 ROS 2 demo. Delivers ML2: a parameterized xacro
differential-drive robot, its TF tree, and an x86-only RViz development config.

**Runs on:** x86 workstation only. RViz2 is an OGRE 2 / desktop-OpenGL
application and must never be placed on the Aquila AM69 — see `CLAUDE.md` rule 1.

## Why a differential-drive robot and not a quadruped

The demo's target robot is a quadruped, but ML2/ML3 ship a diff-drive base
deliberately. As of August 2026 no maintained project delivers
quadruped + ROS 2 Jazzy + Gazebo Harmonic + working Nav2:

- `chvmp/champ` upstream is ROS 1 only (last commit July 2024, `move_base`/`amcl`).
- The best Jazzy + Harmonic CHAMP base (`khaledgabr77/unitree_go2_ros2`) lists
  Nav2 as "coming soon" and has since May 2025.
- The only demonstrated CHAMP + Nav2 stack is ROS 2 Humble + **Gazebo Classic**,
  and compensates for odometry error by doubling linear velocity in the state
  estimator — a workaround, not a calibration.

Gait-kinematics dead reckoning drifts far more than wheel odometry, and Nav2
localization depends on decent odometry. Taking that on first would have blocked
containers, arm64 emulation, and hardware bring-up — the actual point of the demo.

The project topic contract exists precisely to make this reversible: everything
downstream speaks `/demo/cmd_vel` (`geometry_msgs/Twist`), so swapping in a
quadruped later is confined to `demo_description` and `demo_simulation`. No Nav2,
perception, HMI, or container change is required. Tracked as ML3.5.

## TF ownership

Duplicate publishers for one transform are the single most common cause of
jitter and unexplained localization failure. Exactly one owner per transform:

| Transform | Owner | Machine | Mechanism |
| --- | --- | --- | --- |
| `base_footprint → base_link`<br>`base_link → {laser_frame, camera_link, wheels}` | `robot_state_publisher` | x86 host | Computed from this package's URDF. Wheel angles arrive on `/joint_states`, published by Gazebo's `JointStatePublisher` system plugin (ML3) or by `joint_state_publisher_gui` (ML2). |
| `odom → base_footprint` | Gazebo `DiffDrive` system plugin | x86 host, inside `gz sim` | **Not** `robot_state_publisher`'s job — RSP never knows the odom frame. Configured in `demo_robot.urdf.xacro`; reaches ROS 2 through `ros_gz_bridge` (ML3). |
| `map → odom` | Nav2 `amcl` | x86 host (ML3); Aquila from M2 | Standard AMCL behavior once a map is loaded. |

In ML2 there is no `map` frame and no `odom` frame. That is correct: neither
simulation nor localization is running yet. The RViz fixed frame is
`base_footprint`.

## Frame tree

```
base_footprint            ground projection; Nav2's robot_base_frame
  └── base_link           chassis origin, one wheel-radius above ground
        ├── laser_frame   2D lidar   → /demo/scan
        ├── camera_link   RGB camera → /demo/camera/image_raw
        ├── left_wheel_link    (continuous, driven)
        ├── right_wheel_link   (continuous, driven)
        └── caster_wheel_link  (fixed, passive)
```

`base_footprint` is the **parent** of `base_link`, following the standard ROS
convention used by `turtlebot3` and `nav2_minimal_tb4`. The tree diagram in
`.ai/AGENTS.md` §5.3 is ambiguous on nesting; the documented invariant is that
both frames exist and connect correctly.

## Parameters

Every dimension is a `xacro:arg` — nothing is hardcoded. Override at expansion:

```bash
xacro demo_robot.urdf.xacro wheel_separation:=0.40 lidar_range_max:=8.0
```

| Group | Args |
| --- | --- |
| Identity | `robot_name` |
| Chassis | `chassis_length`, `chassis_width`, `chassis_height`, `chassis_mass` |
| Wheels | `wheel_radius`, `wheel_width`, `wheel_separation`, `wheel_mass`, `wheel_base_offset_x` |
| Caster | `caster_radius`, `caster_mass` |
| Lidar | `lidar_offset_x`, `lidar_offset_z`, `lidar_samples`, `lidar_range_max` |
| Camera | `camera_offset_x`, `camera_offset_z`, `camera_width`, `camera_height` |

`wheel_separation` is the critical one: it is consumed both by the geometry and
by the Gazebo `DiffDrive` plugin. If the two disagree, Gazebo reports odometry
that does not match physical motion, and Nav2 localization degrades in a way
that is very hard to trace back to the URDF. `test_urdf_parses.py` asserts they
match.

`robot_name` must equal the `-name` passed to `ros_gz_sim create` in ML3, because
the DiffDrive plugin scopes its topics under `/model/<name>/`.

## Gazebo plugin choice

The model uses the native `gz-sim-diff-drive-system` plugin rather than
`gz_ros2_control`. The `.so` already ships with the host's Gazebo Harmonic
install, so it costs no extra packages, whereas `gz_ros2_control` pulls in
`ros2_control`, `controller_manager`, and `diff_drive_controller` — a much larger
surface for a milestone whose acceptance is "teleop works, topics exchange
messages, Nav2 activates, a goal completes". It is also the pattern used by
`nav2_minimal_tb*_sim` and Gazebo's own `diff_drive.sdf`.

Moving to `gz_ros2_control` later (likely with the quadruped, which needs
joint-level control) changes nothing downstream — that is what the topic
contract is for.

The `<gazebo>` blocks are inert in ML2; RViz ignores them.

## Sensor topic naming

Each `<sensor>` sets an explicit short `<topic>` (`scan`, `camera/image_raw`).
Without it Gazebo auto-generates a scoped name like
`/world/warehouse/model/demo_robot/link/laser_frame/sensor/lidar/scan`, which
embeds the world name and breaks the bridge config whenever the world changes.

Each sensor also sets `<gz_frame_id>` to match its URDF link name. A mismatch
means messages arrive in ROS 2 tagged with a frame the TF tree does not contain,
and every costmap silently drops them.

Confirm the actual names with `gz topic -l` while `gz sim` is running before
binding them in `demo_simulation/config/bridge_warehouse.yaml`.

## Usage

```bash
cd ros2_ws && colcon build --symlink-install --packages-select demo_description
source install/setup.bash

# Expand and inspect
xacro $(ros2 pkg prefix demo_description)/share/demo_description/urdf/demo_robot.urdf.xacro > /tmp/demo_robot.urdf
check_urdf /tmp/demo_robot.urdf            # needs liburdfdom-tools

# Visualize (x86 only) — wheels rotate with the GUI sliders
ros2 launch demo_description view_robot.launch.py

# TF tree
ros2 run tf2_tools view_frames

# Tests
colcon test --packages-select demo_description && colcon test-result --verbose
```

## Tests

`test_urdf_parses.py` asserts: xacro expands; the URDF parses; all required links
exist; the tree has exactly one root (`base_footprint`); wheel joints are
continuous; sensors are fixed to `base_link`; no link has degenerate mass or
inertia (the classic first-run `gz sim` failure); and the DiffDrive
`wheel_separation` matches the actual track width.

Plus `test_flake8.py` and `test_pep257.py`, matching `demo_tutorials`.
