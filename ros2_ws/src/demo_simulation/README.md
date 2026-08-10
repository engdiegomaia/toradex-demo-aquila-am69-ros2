# demo_simulation

Gazebo Harmonic world, robot spawn, `ros_gz_bridge` mappings, and keyboard
teleop. Delivers the simulation half of ML3.

**Runs on:** x86 workstation only, in **all three modes**. Gazebo is an OGRE 2 /
desktop-OpenGL application and the AM69 GPU exposes only OpenGL ES 3.2 and
Vulkan 1.2 — see `CLAUDE.md` rule 1. In `target` mode this package still runs on
the host; only navigation and perception move to the module.

## Setup — the world is not vendored

The warehouse world comes from `nav2_minimal_tb4_sim`, maintained by the Nav2
organization and native Harmonic SDF. It is installed as a system package rather
than copied into this repo:

```bash
sudo apt install -y ros-jazzy-nav2-minimal-tb4-sim
```

Then either copy the world into `worlds/`, or point the launch at the installed
copy:

```bash
ros2 launch demo_simulation simulation.launch.py \
    world:=$(ros2 pkg prefix nav2_minimal_tb4_sim)/share/nav2_minimal_tb4_sim/worlds/depot.sdf
```

The AWS RoboMaker warehouse world was archived in July 2026 and is Gazebo
Classic — do not use it.

## Topic bridge

`config/bridge_warehouse.yaml` is the whole ROS↔Gazebo boundary. Every ROS-side
name lives under `/demo`, per the topic contract.

| Gazebo | ROS 2 | Direction |
| --- | --- | --- |
| `/clock` | `/clock` | gz → ros |
| `/model/demo_robot/cmd_vel` | `/demo/cmd_vel` | ros → gz |
| `/model/demo_robot/odom` | `/demo/odom` | gz → ros |
| `/model/demo_robot/tf` | `/tf` | gz → ros |
| `/world/warehouse/model/demo_robot/joint_state` | `/joint_states` | gz → ros |
| `/scan` | `/demo/scan` | gz → ros |
| `/camera/image_raw` | `/demo/camera/image_raw` | gz → ros |
| `/camera/camera_info` | `/demo/camera/camera_info` | gz → ros |

Two naming traps live here:

- **Sensor topics are short by choice.** The `<topic>` values in
  `demo_description/urdf/_sensors.xacro` override Gazebo's auto-generated scoped
  names like `/world/warehouse/model/demo_robot/link/laser_frame/sensor/lidar/scan`,
  which embed the world name and break this file the moment the world changes.
- **Plugin topics are scoped and cannot be shortened.** `DiffDrive` and
  `JointStatePublisher` publish under `/model/<robot_name>/...`, which is why
  `robot_name` must match the `-name` passed to `ros_gz_sim create`. The
  `joint_state` entry additionally embeds the **world name** — if you switch
  away from a world called `warehouse`, that one line must change.

Always confirm the gz side with `gz topic -l` while the simulator runs. A name
mismatch is silent: the bridge starts fine and the topic simply never publishes.

## TF ownership

This package publishes no transforms itself. It starts
`robot_state_publisher` (fixed edges, from the URDF) and bridges
`odom → base_footprint` out of the `DiffDrive` plugin. `map → odom` belongs to
AMCL in `demo_navigation`. Full table in `demo_description/README.md`.

## Running

```bash
# terminal 1 — simulator, robot, bridge
ros2 launch demo_simulation simulation.launch.py

# terminal 2 — drive it
ros2 launch demo_simulation teleop.launch.py
```

Headless, for CI:

```bash
ros2 launch demo_simulation simulation.launch.py gui:=false
```

Arguments: `world`, `robot_name`, `x`, `y`, `yaw`, `gui`, `model`.

### Teleop needs a TTY

`teleop_twist_keyboard` reads keys from the focused terminal, and a
launch-started node has no attached TTY — hence `prefix='xterm -e'`. Without
xterm installed, run the node directly instead:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/demo/cmd_vel
```

Do not run teleop and Nav2 simultaneously unless you want to watch them fight
over `/demo/cmd_vel`.

## Verifying the graph

```bash
ros2 topic list | grep demo
ros2 topic hz /demo/scan          # ~10 Hz
ros2 topic hz /demo/camera/image_raw   # ~15 Hz
ros2 topic echo /demo/odom --once
ros2 run tf2_tools view_frames
```
