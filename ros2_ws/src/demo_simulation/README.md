# demo_simulation

Gazebo Harmonic worlds, robot spawn, `ros_gz_bridge` topic mappings, the
Go2 gait adapter, and the sim-control / scene-camera façades used by the
web cockpit.

**Runs on:** x86 workstation only, in every mode. Gazebo is an OGRE 2 /
desktop-OpenGL application, and the Aquila AM69 GPU exposes only OpenGL ES 3.2
and Vulkan 1.2. In `hil` mode this package still runs entirely on the host —
only navigation and perception move to the module.

## Overview

Two independent plants live behind this package, selected by
`ROBOT_TYPE=quadruped|diffdrive` (default `quadruped`) through
`demo_bringup/demo_bringup/robot_selection.py`: `quadruped.launch.py` spawns
the Unitree Go2 (vendored `go2_description` + `unitree_guide_controller`
gait FSM) in a maze world, and `simulation.launch.py` spawns the
differential-drive robot from `demo_description` in the apt-provided
warehouse world. Every ROS-side topic name lives under `/demo`, per the
project's topic contract; the plant selector uses one `ros_gz_bridge` mapping
file per robot (`config/bridge_quadruped.yaml`, `config/bridge_warehouse.yaml`).

## Nodes

| Node | Publishes | Subscribes | Services |
| --- | --- | --- | --- |
| `twist_to_inputs` | `/control_input` (`control_input_msgs/Inputs`) | `/demo/cmd_vel` | offers Trigger `/demo/gait/hold`, `/demo/gait/resume` |
| `clock_throttle` | `/clock` | `/demo/clock_raw` (configurable) | — |
| `scene_view_controller` | `/demo/cockpit/scene/following` (Bool) | `/demo/cockpit/scene/cmd_view` (TwistStamped), `/demo/odom` | offers Trigger `/demo/cockpit/scene/reset_view`, SetBool `/demo/cockpit/scene/follow`; calls `SetEntityPose` `/demo/sim/set_entity_pose` |
| `sim_control_relay` | — | — | offers Trigger `/demo/sim/{play,pause,reset}`; calls `ControlWorld` `/demo/sim/control`, `SetEntityPose`, `/demo/gait/{hold,resume}` |
| `maze_escape_validator` | `/demo/maze/escaped` (Bool, transient-local) | `/demo/odom` | — |

| Parameter | Default | Node | Description |
| --- | --- | --- | --- |
| — | — | `twist_to_inputs` | No declared parameters; command timeout (`0.3 s`), control period (`0.05 s`) and stick clamp (`±0.5`) are module constants matched to the vendored gait controller. |
| `rate_hz` | `100.0` | `clock_throttle` | Output `/clock` rate; `0` means passthrough. Not on the Go2 path — `bridge_quadruped.yaml` bridges `/clock` directly. |
| `input_topic` | `'/demo/clock_raw'` | `clock_throttle` | Upstream (unthrottled) clock source. |
| `iso_x/y/z`, `iso_pitch/yaw` | `-3.0 / 3.0 / 2.4`, `0.515 / -0.785` | `scene_view_controller` | Default isometric scene-camera pose. |
| `top_x/y/z`, `top_pitch/yaw` | `0.0 / 0.0 / 6.0`, `π/2 / π/2` | `scene_view_controller` | Default top-down scene-camera pose. |
| `follow` | `true` | `scene_view_controller` | Whether the scene camera tracks the robot. |
| `follow_topic` | `'/demo/odom'` | `scene_view_controller` | Pose source used for following. |
| `follow_offset_x/y/yaw` | `0.0` / `0.0` / `0.0` | `scene_view_controller` | Camera offset relative to the followed pose. |
| `robot_name` | `'demo_robot'` | `sim_control_relay` | Entity reset by `/demo/sim/reset`. |
| `spawn_x/y/z/yaw` | `0.0 / 0.0 / 0.5 / 0.0` | `sim_control_relay` | Pose applied on reset. |
| `world` | `''` | `maze_escape_validator` | Active only for the `maze11` world; empty elsewhere. |

## Launch files

| File | Starts | Key arguments |
| --- | --- | --- |
| `launch/simulation.launch.py` | Diff-drive plant: `gz sim`, `robot_state_publisher`, spawn, `ros_gz_bridge` (`bridge_warehouse.yaml`), scene cameras, sim control | `world`, `robot_name`, `x`, `y`, `yaw`, `gui`, `use_meshes`, `model` |
| `launch/quadruped.launch.py` | Go2 plant: `gz sim`, `robot_state_publisher`, spawn, controller spawners (`joint_state_broadcaster`, IMU, `unitree_guide_controller` + `gait_go2.yaml`), `ros_gz_bridge` (`bridge_quadruped.yaml`), `twist_to_inputs`, `maze_escape_validator`, scene cameras, sim control | `world`, `robot_name`, `gait_params`, `jsb_params`, `x`, `y`, `yaw`, `height`, `gui`, `scene_cameras` |
| `launch/scene_cameras.launch.py` | Spawns iso/top scene cameras (`models/cockpit_scene_{iso,top}.sdf`) + `scene_view_controller` | `scene_cameras`, `world`, `follow`, `follow_offset_x/y/yaw`, per-camera pose overrides |
| `launch/sim_control.launch.py` | `ControlWorld`/`SetEntityPose` service bridge + `sim_control_relay` | — |
| `launch/teleop.launch.py` | `teleop_twist_keyboard` → `/demo/cmd_vel`, in an `xterm` (needs a TTY) | `speed`, `turn` |

## Configuration

| File | What |
| --- | --- |
| `config/bridge_quadruped.yaml` | `ros_gz_bridge` mapping for the Go2 (`/clock` bridged directly). |
| `config/bridge_warehouse.yaml` | `ros_gz_bridge` mapping for the diff-drive plant, against the apt-provided `nav2_minimal_tb4_sim` warehouse world. |
| `config/gait_go2.yaml` | Gait tuning passed to the controller spawner via `--param-file`, kept out of `go2_description` so that package stays byte-identical to upstream. |
| `config/joint_state_broadcaster.yaml` | Decimates `/joint_states` from 1000 Hz to 50 Hz. |
| `urdf/go2_sim.urdf.xacro` | Simulation wrapper around the vendored `go2_description` (adds camera, lidar, `/demo/odom` ground-truth publisher). |
| `worlds/quadruped_{empty,ramp,rough,corridor,objects,maze11}.sdf` | Scenario worlds — see `docs/architecture.md` / `docs/engineering-log.md` for what each one validates. |
| `models/cockpit_scene_{iso,top}.sdf` | Spawnable scene cameras used by the cockpit. |

## Build and test

```bash
cd ros2_ws
colcon build --symlink-install --packages-select demo_simulation
colcon test --packages-select demo_simulation && colcon test-result --verbose
```

## Running

```bash
# terminal 1 — simulator, robot, bridge (quadruped is the default robot)
ros2 launch demo_simulation quadruped.launch.py

# terminal 2 — drive it
ros2 launch demo_simulation teleop.launch.py

# headless, e.g. for CI
ros2 launch demo_simulation quadruped.launch.py gui:=false
```

The host-side helper `scripts/run_quadruped_sim.sh` builds and launches the
quadruped plant inside its own container image, for HIL bring-up without a
full workspace source on the host.

## Verifying the graph

```bash
ros2 topic list | grep demo
ros2 topic hz /demo/scan
ros2 topic hz /demo/camera/image_raw
ros2 topic echo /demo/odom --once
ros2 run tf2_tools view_frames
```

## Notes

- **The Go2's `/demo/odom` is a temporary ground-truth source, not an
  estimator.** `go2_sim.urdf.xacro` publishes it from exact Gazebo model
  pose; the model's own ground-truth TF topic is deliberately not bridged, to
  avoid a duplicate `odom → base` publisher. Legged-state estimation will
  replace this producer and own that edge; `demo_bringup/odom_tf` documents
  the same boundary from the consumer side.
- **Every Gazebo-side topic name used by the DiffDrive/JointStatePublisher
  plugins is unscoped on purpose** (`/cmd_vel`, `/odom`, `/tf`,
  `/joint_states`), matching the literal `<topic>` elements in the plugin
  blocks. Gazebo *also* advertises scoped defaults
  (`/model/demo_robot/{cmd_vel,odom,tf}`) that look more "correct" in
  `gz topic -l` but have nothing attached — use `gz topic -i -t <name>` to
  tell a live publisher from a decoy.
- **Startup is timed, not immediate.** `simulation.launch.py` delays the
  spawner to t=12 s and the bridge to t=15 s: `ros_gz_sim create` calls
  Gazebo's world-listing service, which does not exist at t=0 and retries
  forever instead of failing. The failure mode is deceptive — the robot
  appears spawned and sensors publish, but the drive plugins never
  initialize and odometry stays silent. Raise both timers (and the matching
  ones in `demo_bringup/launch/learn.launch.py`) on a slower machine.
- **`/demo/cmd_vel` is stick units on the quadruped path, not SI** — see
  `demo_bringup/README.md`. `twist_to_inputs` converts it to
  `control_input_msgs/Inputs` for the gait controller and ages commands out
  after 0.3 s of silence, which is also the mechanism that lets settling
  (not publishing) act as the stop command.
- **`worlds/quadruped_empty.sdf` is the deterministic, asset-free
  integration fixture** — it still includes Gazebo's Sensors system, unlike
  the built-in `empty.sdf`, so the camera and lidar actually publish.
- **Do not run teleop and Nav2 at the same time** — both can publish
  `/demo/cmd_vel`, and the robot moves in jitters neither log explains.
