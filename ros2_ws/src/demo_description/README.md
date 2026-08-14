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
| `base_link → base_footprint`<br>`base_link → {rplidar_link, oakd_*, imu_link, wheels}` | `robot_state_publisher` | x86 host | Computed from this package's URDF. Wheel angles arrive on `/joint_states`, published by Gazebo's `JointStatePublisher` system plugin (ML3) or by `joint_state_publisher_gui` (ML2). |
| `odom → base_link` | Gazebo `DiffDrive` system plugin | x86 host, inside `gz sim` | **Not** `robot_state_publisher`'s job — RSP never knows the odom frame. Hardcoded upstream in `icreate/create3.urdf.xacro`; reaches ROS 2 through `ros_gz_bridge` (ML3). |
| `map → odom` | Nav2 `amcl` (or `slam_toolbox`) | x86 host (ML3); Aquila from M2 | Standard AMCL behavior once a map is loaded. |

In ML2 there is no `map` frame and no `odom` frame. That is correct: neither
simulation nor localization is running yet. The RViz fixed frame is `base_link`.

## Frame tree

The robot is the upstream TurtleBot 4 (`nav2_minimal_tb4_description`); this
package only wraps it. See the header of `urdf/demo_robot.urdf.xacro` for why.

```
base_link                       ROOT; Nav2's robot_base_frame
  ├── base_footprint            IDENTITY transform (xyz 0 0 0, rpy 0 0 0)
  ├── shell_link
  │     ├── rplidar_link                        2D lidar → /demo/scan
  │     └── oakd_camera_bracket → oakd_link
  │           └── oakd_rgb_camera_frame → …_optical_frame
  │                                             RGBD    → /demo/camera/image_raw
  ├── imu_link                                  IMU     → /demo/imu
  ├── left_wheel / right_wheel  (continuous, driven)
  └── front_caster_link         (fixed, passive)
```

**Note the inversion.** Upstream makes `base_link` the root with
`base_footprint` as its *child* — the opposite of the usual ROS convention and of
this package's earlier hand-built model. Nav2 therefore uses `base_link` as
`robot_base_frame`, because upstream's `DiffDrive` hardcodes that child frame with
no xacro arg to override it. The two frames coincide numerically
(`base_footprint_joint` is an identity transform), and
`test/test_urdf_parses.py::test_base_footprint_coincides_with_base_link` fails if
an upstream bump ever breaks that equivalence.

Historical note — the old hand-built model made `base_footprint` the **parent** of `base_link`, following the standard ROS
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
| Compatibility | `use_meshes` (accepted, **ignored**) |

The long list of dimension args (`chassis_*`, `shell_*`, `wheel_*`, `lidar_*`,
`camera_*`, `standoff_*`, …) is **gone**. Those existed to parameterize the
hand-built assembly; geometry now comes from the upstream model and is not
re-parameterized here. To change a dimension, change it upstream or fork the
upstream xacro — do not reintroduce local offsets, which is what scattered the
parts before.

`use_meshes` is accepted and ignored, so existing launch files and documented
commands that pass it keep working instead of dying on an unknown-argument error.
Upstream is mesh-only; there is no primitives fallback to switch to. Nothing on
the module side depends on it — no OGRE 2 application ever runs on the Aquila, so
the module never renders this model at all.

`wheel_separation` (0.233 m) and `wheel_radius` (0.03575 m) remain the critical
values: they are consumed both by the geometry and by the Gazebo `DiffDrive`
plugin. If the two disagree, Gazebo reports odometry that does not match physical
motion, and Nav2 localization degrades in a way that is very hard to trace back to
the URDF. Inheriting both from upstream keeps them locked to the meshes by
definition, and `test_urdf_parses.py` still asserts they match.

Note that `robot_name` does **not** scope the DiffDrive gz topics — `<topic>`,
`<odom_topic>` and `<tf_topic>` are literal. See the note in
`demo_simulation/config/bridge_warehouse.yaml`.

## Visuals: meshes and the primitive fallback

Visuals come from `nav2_minimal_tb4_description` DAE meshes, referenced via
`package://` and **not vendored** — ~25 MB of binaries that do not belong in
git. The package is Apache-2.0, Nav2-maintained, and already required for the
warehouse world.

```bash
ros2 launch demo_description view_robot.launch.py                              # meshes
ros2 launch demo_description view_robot.launch.py xacro_args:="use_meshes:=false"
```

Two rules when editing `_visuals.xacro`:

1. **Visual and collision are different geometry on purpose.** Visual is a
   mesh; collision is a primitive. A mesh collision makes physics far more
   expensive for no gain — a cylinder describes a round chassis well.
2. **Keep the `use_meshes:=false` path working.** It is the escape hatch for a
   host without the package, and what an ML4 arm64 image uses if the meshes are
   dropped. Collision, inertia, TF and odometry are identical either way.

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
