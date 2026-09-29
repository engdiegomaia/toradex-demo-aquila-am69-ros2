# demo_description

Robot model for the diff-drive path of the demo: a parameterized xacro
wrapping the upstream TurtleBot 4 description, its TF tree, and an
x86-only RViz viewer.

**Runs on:** x86 host only. `view_robot.launch.py` starts RViz2, an OGRE 2 /
desktop-OpenGL application that must never run on the Aquila AM69 — the module
GPU exposes only OpenGL ES 3.2 and Vulkan 1.2. The URDF itself is
architecture-neutral and is also consumed (via `robot_state_publisher`) inside
the `sim` container on the host.

## Overview

The demo's target robot is the simulated Unitree Go2 quadruped
(`go2_description` / `demo_simulation`); this package provides the
differential-drive fallback path used in earlier milestones and kept live as
the warehouse-scenario baseline. It wraps `nav2_minimal_tb4_description`
(Apache-2.0, Nav2-maintained) rather than re-deriving the model, so geometry,
meshes and the `DiffDrive`/`JointStatePublisher` plugin wiring stay locked to
what upstream ships. This package contributes no ROS nodes — it is a
model-and-launch package, consumed by `demo_simulation` and `demo_navigation`.

## Frame tree

```
base_link                       ROOT; Nav2's robot_base_frame
  ├── base_footprint             identity transform (xyz 0 0 0, rpy 0 0 0)
  ├── shell_link
  │     ├── rplidar_link                        2D lidar → /demo/scan
  │     └── oakd_camera_bracket → oakd_link
  │           └── oakd_rgb_camera_frame → …_optical_frame
  │                                             RGBD    → /demo/camera/image_raw
  ├── imu_link                                  IMU     → /demo/imu
  ├── left_wheel / right_wheel   (continuous, driven)
  └── front_caster_link          (fixed, passive)
```

`base_link` is the root, with `base_footprint` as its child — the reverse of
the usual ROS convention, inherited from upstream because `DiffDrive`
hardcodes that child frame with no xacro override. The two frames coincide
numerically; `test_urdf_parses.py` pins that equivalence.

## TF ownership

| Transform | Owner | Mechanism |
| --- | --- | --- |
| `base_link → {base_footprint, rplidar_link, oakd_*, imu_link, wheels}` | `robot_state_publisher` | Computed from this package's URDF; wheel angles arrive on `/joint_states`. |
| `odom → base_link` | Gazebo `DiffDrive` system plugin | Not `robot_state_publisher`'s job — bridged from Gazebo via `ros_gz_bridge`. |
| `map → odom` | Nav2 `amcl` | Standard AMCL behavior once a map is loaded. |

## Launch files

| File | What | Key arguments |
| --- | --- | --- |
| `launch/view_robot.launch.py` | `robot_state_publisher` + `joint_state_publisher_gui` + `rviz2`, for standalone model inspection | `model`, `gui`, `rviz`, `xacro_args` |

## Configuration

| File | What |
| --- | --- |
| `urdf/demo_robot.urdf.xacro` | The wrapped, parameterized xacro. Dimension args are inherited from upstream, not re-declared locally — see Notes. |
| `rviz/demo_description.rviz` | Viewer-only RViz config (no `map` frame); not a substitute for `demo_bringup/rviz/demo_view.rviz`. |
| `scripts/weld_fixed_joints.py` | Installed here but invoked by `demo_simulation`'s launch file via a share-path lookup; without it Gazebo spawns the robot as loose physics bodies. |

## Build and test

```bash
cd ros2_ws
colcon build --symlink-install --packages-select demo_description
source install/setup.bash
colcon test --packages-select demo_description && colcon test-result --verbose
```

```bash
# Expand and inspect
xacro $(ros2 pkg prefix demo_description)/share/demo_description/urdf/demo_robot.urdf.xacro > /tmp/demo_robot.urdf
check_urdf /tmp/demo_robot.urdf     # needs liburdfdom-tools

# Visualize (x86 only)
ros2 launch demo_description view_robot.launch.py
```

`test_urdf_parses.py` asserts: xacro expands and the URDF parses; all required
links exist with exactly one root (`base_footprint`); wheel joints are
continuous; sensors are fixed to `base_link`; no link has degenerate mass or
inertia; and `wheel_separation` matches the actual track width.

## Notes

- **`wheel_separation` (0.233 m) and `wheel_radius` (0.03575 m) are the
  critical values.** They feed both the geometry and the Gazebo `DiffDrive`
  plugin; if the two disagree, odometry stops matching physical motion and
  Nav2 localization degrades in a way that is hard to trace back to the
  URDF. Both are inherited from upstream, not re-parameterized here.
- **Visuals are not vendored.** DAE meshes are referenced via `package://` from
  `nav2_minimal_tb4_description` (`sudo apt install ros-jazzy-nav2-minimal-tb4-description`
  transitively, or install the meta-dependency directly). `use_meshes:=false`
  is accepted as a compatibility no-op — upstream is mesh-only, and nothing on
  the arm64 side ever renders this model.
- **The native `gz-sim-diff-drive-system` plugin is used, not
  `gz_ros2_control`.** It ships with Gazebo Harmonic at no extra cost and
  matches the pattern used by `nav2_minimal_tb*_sim`. Moving to
  `gz_ros2_control` later (as the quadruped path does) changes nothing
  downstream — that is what the `/demo/*` topic contract is for.
- **Sensor topics are explicit, short names** (`scan`,
  `camera/image_raw`), not Gazebo's auto-generated, world-scoped defaults.
  Confirm actual names with `gz topic -l` before binding them in
  `demo_simulation/config/bridge_warehouse.yaml`.
- **`robot_name` does not scope the DiffDrive Gazebo topics** — `<topic>`,
  `<odom_topic>` and `<tf_topic>` in the plugin blocks are literal, unscoped
  names (`/cmd_vel`, `/odom`, `/tf`), not `/model/<name>/...`. See the note in
  `demo_simulation/config/bridge_warehouse.yaml`.
