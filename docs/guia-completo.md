# Complete guide — operation and web cockpit

How to run the demo, how to modify it without breaking what already works, and
how to use the web cockpit that comes with it. Two guides that used to be
separate files (`guia-operacao.md` and `guia-cockpit.md`) and were unified here
on 26/08/2026 — the content did not change, it just stopped being scattered.

Written for someone arriving at the project with no prior ROS 2 experience.
Where a known trap exists, it is described at the point where you would hit it —
not in a troubleshooting section at the end.

---

## Table of contents

**[Part I: Demo operation](#part-i-demo-operation)**

1. [Before you start](#1-before-you-start)
2. [Running the demo](#2-running-the-demo)
3. [How the project fits together](#3-how-the-project-fits-together)
4. [Editing the robot](#4-editing-the-robot)
5. [Editing navigation](#5-editing-navigation)
6. [Editing perception](#6-editing-perception)
7. [Editing the simulation](#7-editing-the-simulation)
8. [Diagnostics](#8-diagnostics)
9. [The traps that have already cost time](#9-the-traps-that-have-already-cost-time)
10. [The Aquila AM69 module](#10-the-aquila-am69-module)

**[Part II: Web cockpit](#part-ii-web-cockpit)**

1. [What the cockpit is](#1-what-the-cockpit-is)
2. [Running it](#2-running-it)
3. [The screen](#3-the-screen)
4. [The controls](#4-the-controls)
5. [What you CANNOT do](#5-what-you-cannot-do)
6. [Switching scenario](#6-switching-scenario)
7. [Tuning image quality](#7-tuning-image-quality)
8. [Cockpit diagnostics](#8-cockpit-diagnostics)
9. [The cockpit traps that have already cost time](#9-the-cockpit-traps-that-have-already-cost-time)
10. [How the cockpit is built](#10-how-the-cockpit-is-built)
11. [What is done and what is missing](#11-what-is-done-and-what-is-missing)

---

# Part I: Demo operation

How to run the demo and how to modify it without breaking what already works.

Written for someone arriving at the project with no prior ROS 2 experience.
Where a known trap exists, it is described at the point where you would hit it —
not in a troubleshooting section at the end.

**Current phase: L3 complete.** Sections 1 through 9 describe the demo running
on the x86 host, natively, without containers. [Section 10](#10-the-aquila-am69-module)
is the exception: it covers the Aquila AM69 module, which is always container
and always `arm64`.

---

## 1. Before you start

### Where each thing runs

Half of the possible errors in this project come from putting something on the
wrong machine. The rule does not change once the containers arrive:

| Component | Machine | Why |
| --- | --- | --- |
| Gazebo, RViz2 | **x86 host** | They are OGRE 2 / desktop OpenGL. The AM69 GPU only exposes OpenGL ES 3.2 and Vulkan 1.2. |
| Nav2, perception, bringup | Aquila (or host, on L3) | No rendering. |
| Chromium HMI | Aquila | GPU accelerated, but through ES. |

Never place Gazebo or RViz2 in a service that starts on the module. This is not
a style preference: the module does not have the driver for it.

### Installation

```bash
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-minimal-tb4-sim \
    ros-jazzy-nav2-minimal-tb4-description \
    ros-jazzy-joint-state-publisher-gui \
    liburdfdom-tools
```

The last two `nav2-minimal-tb4-*` packages are not optional as far as
appearance goes: they provide the warehouse world **and** the robot meshes. Both
are Apache-2.0, maintained by the Nav2 organization.

### Building

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` makes launch files and YAMLs be read straight from `src/` —
you edit and run again, without rebuilding. **This only holds for data files.**
If you changed a node's Python code, rebuild.

> **Never commit `build/`, `install/` or `log/`.** With `--symlink-install`
> these directories contain absolute paths from your machine. It already
> happened in this repo: four symlinks pointing at a directory that no longer
> existed broke the build for anyone who cloned it. `.gitignore` covers this.

### The `source` everybody forgets

Every new terminal needs:

```bash
source /opt/ros/jazzy/setup.bash
source ~/toradex/demo/aquila-am69-ros2/ros2_ws/install/setup.bash
```

Without the second one, `ros2 launch demo_bringup ...` answers "package not
found" even with the package built.

---

## 2. Running the demo

### Everything at once

```bash
ros2 launch demo_bringup learn.launch.py
```

It starts Gazebo, the robot, the bridge, perception, Nav2 and RViz2. **It takes
~30 s until Nav2 is ready** — and that is on purpose:

```
t=0s    Gazebo starts loading the world (~10 s of meshes)
t=12s   robot spawn
t=15s   ros_gz_bridge
t=20s   perception
t=25s   Nav2 + RViz2
```

Those timers are not lazy slack; see [section 9](#9-the-traps-that-have-already-cost-time).
On a slower machine, increase **all of them together**.

To send the robot to a destination: in RViz2, the **2D Goal Pose** button, click
and drag on the map.

### Demonstration cockpit

A web page with the demo's five regions — scene, navigation, logs, camera and
control bar — served by a container and opened in the browser:

```bash
cd docker
docker compose -f compose.host.yml --profile learn up -d --build
# open http://localhost:8081
```

**The complete guide is in [Part II](#part-ii-web-cockpit) of this document**:
what each panel shows, what the buttons do, how to switch scenario, how to tune
image quality, and its own traps.

> **The old path was discarded.** There used to be a procedure here that tried
> to **embed X11 windows** of Gazebo, RViz and `rqt_image_view` into a single
> application (`./scripts/run_cockpit.sh`). Four attempts, none accepted: in the
> last trial RViz and the camera stayed external and the internal panels were
> empty. **Do not resume that path** — the checkpoint is in
> [`results/cockpit-standalone-parcial.md`](results/cockpit-standalone-parcial.md),
> marked as superseded.
>
> The underlying reason was not an implementation one: RViz and Gazebo are
> OGRE 2 and need desktop OpenGL, so they could never go to the Aquila (rule 1
> of `CLAUDE.md`). That cockpit would never become the module's HMI. The web
> cockpit, which renders from ROS 2 topics, does.

### Useful options

```bash
# Without Nav2 — simulation only, for teleoperation
ros2 launch demo_bringup learn.launch.py navigation:=false

# Without RViz2 (headless, CI)
ros2 launch demo_bringup learn.launch.py rviz:=false

# Another world
ros2 launch demo_bringup learn.launch.py \
    world:=$(ros2 pkg prefix nav2_minimal_tb4_sim)/share/nav2_minimal_tb4_sim/worlds/depot.sdf
```

### Isolated pieces

Useful for debugging: if something fails as a whole, bring up one layer at a
time.

```bash
# Simulator only
ros2 launch demo_simulation simulation.launch.py

# Keyboard teleoperation (separate terminal, requires focus)
ros2 launch demo_simulation teleop.launch.py

# Model only, in RViz — does not require Gazebo
ros2 launch demo_description view_robot.launch.py
```

### Generating a new map

The map in `demo_navigation/maps/warehouse.{yaml,pgm}` is already committed. To
redo it, or to map another world:

```bash
# Terminal 1
ros2 launch demo_simulation simulation.launch.py
# Terminal 2
ros2 launch demo_navigation slam.launch.py
# Terminal 3 — drive the robot through the entire scenario
ros2 launch demo_simulation teleop.launch.py
# Terminal 4 — when the map is complete
ros2 run nav2_map_server map_saver_cli -f meu_mapa
```

Then copy the two files to `demo_navigation/maps/` and pass
`map:=.../meu_mapa.yaml`.

---

## 3. How the project fits together

```
demo_description   the robot: geometry, sensors, Gazebo plugins
demo_simulation    Gazebo + spawn + ROS↔gz bridge + teleoperation
demo_perception    detection stub + costmap adapter
demo_navigation    Nav2, parameters, SLAM, maps
demo_bringup       composition: what starts and in which order
demo_tutorials     L1 exercises, not part of the demo
```

### The topic contract

This is the project invariant. It exists so that NPU inference can arrive later
without refactoring anything:

| Topic | Type | Producer | Consumers |
| --- | --- | --- | --- |
| `/demo/camera/image_raw` | `sensor_msgs/Image` | Gazebo, USB camera or rosbag | `demo_perception` |
| `/demo/perception/detections` | `vision_msgs/Detection2DArray` | `demo_perception` | costmap, HMI |
| `/demo/cmd_vel` | `geometry_msgs/Twist` | Nav2 | Gazebo or real driver |

Two practical consequences when editing:

- **Perception must never know where the image comes from.** If you write
  something into its code that assumes Gazebo, swapping the stub for TIDL stops
  being a container swap and becomes a refactor.
- **The detections feed the costmap, not just the screen.** Even as a stub. If
  that seam breaks, nobody notices until the day real inference arrives.

### Who publishes each transform

A transform with two owners is a robot that shakes in RViz. The table:

| TF edge | Owner |
| --- | --- |
| `map → odom` | `amcl` (Nav2) |
| `odom → base_footprint` | Gazebo DiffDrive plugin |
| `base_footprint → base_link` → sensors | `robot_state_publisher`, from the URDF |

---

## 4. Editing the robot

Files in `ros2_ws/src/demo_description/urdf/`:

| File | Content |
| --- | --- |
| `demo_robot.urdf.xacro` | main: dimensions, assembly, Gazebo plugins |
| `_visuals.xacro` | mesh visuals and the primitive fallback |
| `_wheel.xacro` | drive wheels and caster wheel |
| `_sensors.xacro` | lidar and camera (URDF + gz `<sensor>` block) |
| `_inertia.xacro` | inertia tensors |
| `_materials.xacro` | colors (RViz only) |

### Changing a dimension

Everything is a xacro argument. Nothing is hardcoded:

```bash
# Test without editing the file
ros2 launch demo_description view_robot.launch.py \
    xacro_args:="wheel_separation:=0.30"
```

If you like it, change the `default` of the corresponding `<xacro:arg>`.

> **`wheel_separation` and `wheel_radius` are not cosmetic.** They go literally
> into the DiffDrive plugin, which integrates both into the odometry. Changing
> the mesh without changing these values (or the other way around) produces a
> robot that **visibly slides while reporting a straight line** — and nothing
> raises an error. If you changed one, check the other.

Changed the chassis radius? Update `robot_radius` in both costmaps in
`demo_navigation/config/nav2_params.yaml`. They do not read the URDF.

### Appearance: meshes and the fallback

The robot renders from DAE meshes in `nav2_minimal_tb4_description`
(iRobot Create 3 + TurtleBot 4), referenced via `package://`. **They are not
copied into this repository** — they are ~25 MB of binaries that do not need to
be in git.

```bash
# Default: meshes
ros2 launch demo_description view_robot.launch.py

# Fallback: primitives
ros2 launch demo_description view_robot.launch.py xacro_args:="use_meshes:=false"
```

Two rules when touching this:

1. **Visual and collision are different geometries on purpose.** Visual is a
   mesh (pretty, expensive); collision is a primitive (cylinder/box, cheap).
   Using a mesh as collision makes the physics orders of magnitude more
   expensive with no gain at all — a cylinder describes a round chassis very
   well.
2. **Keep the `use_meshes:=false` path working.** It is the way out on a
   machine without the package installed, and it is what the L4 arm64 image
   will use if the meshes are removed from it. Collision, inertia, TF and
   odometry are identical on both paths; only the rendering changes.

If you swap the meshes, the `rpy` values in `_visuals.xacro` come from the
upstream macros, which are known to render correctly. A wrong rotation draws the
robot lying down — and the symptom is purely visual, because TF, odometry and
costmaps stay correct.

### After any URDF edit

```bash
# Expand and validate the structure
xacro demo_robot.urdf.xacro > /tmp/r.urdf && check_urdf /tmp/r.urdf

# Both paths must expand
xacro demo_robot.urdf.xacro use_meshes:=false > /dev/null && echo "fallback OK"
```

`check_urdf` should show `base_footprint` as the root and the link tree.

---

## 5. Editing navigation

`demo_navigation/config/nav2_params.yaml`, ~13 server blocks. Parameters live in
YAML, **never embedded in code**.

The ones you will touch most:

| Parameter | Where | Effect |
| --- | --- | --- |
| `max_vel_x`, `max_vel_theta` | `controller_server` | demo speed |
| `robot_radius` | both costmaps | must match the chassis |
| `inflation_radius` | `inflation_layer` | distance it keeps from walls |
| `xy_goal_tolerance` | `general_goal_checker` | precision for calling a goal done |

Edited YAML? You do not need to rebuild (`--symlink-install`), but you **do need
to restart Nav2** — parameters are read during the lifecycle configure.

> Restarting here means **launching again** (or recreating the `nav` service).
> The `/demo/nav/reset` described below does not do it: it deliberately does not
> go through `CONFIGURE`, so a new parameter is not read. A reset that "did not
> pick up the change" is the symptom.

### Discarding the goal and clearing the costmaps

```bash
ros2 service call /demo/nav/reset std_srvs/srv/Trigger
```

It serves "the demo got stuck, I want to start over without tearing anything
down": it cancels any goal in progress, empties both costmaps and recycles the
servers through `PAUSE`/`RESUME`. ~6,6 s on the host. It is the same service as
the cockpit's **reiniciar nav** button. It comes up along with both Nav2 paths
(`navigation.launch.py` and `nav_quadruped.launch.py`); `nav_control:=false`
turns it off.

`/demo/nav/cancel` does only the first part, without touching the costmap or the
lifecycle.

> **Do not swap this for `RESET`+`STARTUP` on the `lifecycle_manager`.** That is
> the obvious path and it takes the whole container down with `SIGSEGV` while
> configuring the `route_server` — measured twice, deterministic. See cockpit
> trap 8, [here](#8-reset--startup-on-nav2-kills-the-container-segfault-in-route_server).

> **Do not leave YAML lists empty.** `docks: []` reaches the launch as a Python
> tuple and takes everything down with `Expected 'value' to be one of [float,
> int, str, bool, bytes], but got '()'`. If you do not want the key, **omit
> it** — do not leave it empty.

---

## 6. Editing perception

```
detection_stub.py       generates deterministic synthetic detections
detections_to_cloud.py  Detection2DArray → PointCloud2 for the costmap
```

The adapter exists because Nav2's stock `ObstacleLayer` does not read
`Detection2DArray`. The alternative would be a C++ costmap plugin; the project
convention is C++ only with hardware-measured performance, and nothing has been
measured on the AM69 yet. Swapping later changes neither side.

**Accepted limitation:** a 2D box has no depth. The adapter assumes a fixed
distance and uses a pinhole model for the azimuth. Hence `clearing: false` (the
projection is too coarse to erase a real obstacle seen by the lidar) and
`observation_persistence: 1.0` (the detection expires instead of becoming a
ghost).

When replacing the stub with real inference, the target is: publish
`vision_msgs/Detection2DArray` on `/demo/perception/detections`, consuming
`sensor_msgs/Image` from `/demo/camera/image_raw`. Nothing else needs to change.

---

## 7. Editing the simulation

### The ROS ↔ Gazebo bridge

`demo_simulation/config/bridge_warehouse.yaml` is the entire boundary. Every
name on the ROS side lives under `/demo`.

> **No entry is model-scoped, and that is correct.** The DiffDrive `<topic>`,
> `<odom_topic>` and `<tf_topic>` elements are **literal**: the plugin listens
> on `/cmd_vel` and publishes on `/odom` and `/tf`, without a prefix. Gazebo
> *also* advertises `/model/demo_robot/{cmd_vel,odom,tf}`, which show up in
> `gz topic -l` and **look** like the right names — but have nobody connected to
> them. Pointing the bridge at them gives a robot that does not move and
> odometry that does not publish, **with no error at all**.

Confirm before trusting:

```bash
gz topic -l                    # does it exist?
gz topic -i -t /cmd_vel        # does it have a publisher/subscriber?
```

`No subscribers on topic` is the symptom.

### Switching the world

The default resolves to `nav2_minimal_tb4_sim`. To use another one, pass
`world:=/caminho/absoluto.sdf`. If you put an `.sdf` in
`demo_simulation/worlds/`, it is installed by `setup.py` — but prefer not to
version heavy worlds.

---

## 8. Diagnostics

Bottom-up order. Stop at the first one that fails.

```bash
# 1. Are the nodes alive?
ros2 node list

# 2. Do the topics exist and publish?
ros2 topic list | grep demo
ros2 topic hz /demo/scan          # expected ~10 Hz
ros2 topic hz /demo/odom          # expected ~30 Hz

# 3. Is the TF tree complete and without gaps?
ros2 run tf2_tools view_frames    # generates frames.pdf
ros2 run tf2_ros tf2_echo odom base_footprint

# 4. Did all of Nav2 start? All 7 servers must be `active`
ros2 lifecycle list /planner_server
for n in map_server amcl planner_server controller_server \
         bt_navigator behavior_server velocity_smoother; do
    echo -n "$n: "; ros2 lifecycle get /$n
done

# 5. Does perception reach the costmap?
ros2 topic info /demo/perception/detection_cloud -v   # Subscription count must be 2
```

**Gazebo side** (gz names are not ROS names):

```bash
gz topic -l
gz topic -i -t /odom
gz model --list
```

### Common symptoms

| Symptom | Likely cause |
| --- | --- |
| The robot appears, sensors publish, but it does not move and `/demo/odom` is silent | Spawned too early, or the bridge pointing at a scoped name. See section 9. |
| Nav2 half `active`, half `inactive` | One server failed to configure and the lifecycle manager aborted the rest. Find out which. |
| `slam_toolbox` runs but does not produce a map | It stayed in `unconfigured` — it is a lifecycle node. See section 9. |
| The costmap ignores the detections | `frame_id` outside the TF tree, or `observation_persistence` expiring. |
| Everything hangs waiting for TF | `/clock` is missing. The bridge must be up before any node with `use_sim_time`. |

---

## 9. The traps that have already cost time

All of them share the same signature: **no error message, everything looking
like it works**. They are here because they come back if somebody "cleans up"
the code.

### 1. A spawn at t=0 never completes

`ros_gz_sim create` first calls the world-list service. At t=0 that service does
not exist, and the client **retries every 5 s forever** instead of failing. The
warehouse world takes ~10 s loading meshes.

The robot still shows up in the world — the asynchronous create accepts it — so
`gz model --list` shows `demo_robot` and the sensors publish. But the DiffDrive
and JointStatePublisher plugins **never initialize**.

→ Do not remove the 12 s and 15 s `TimerAction`s in `simulation.launch.py`.

### 2. gz topics are not scoped

Detailed in [section 7](#the-ros--gazebo-bridge). It is the one that looks most
like a configuration bug and consumes the most time.

### 3. `slam_toolbox` is a lifecycle node

It comes up in `unconfigured` and stays there. The process runs, logs normally,
and **creates no scan subscription and publishes no map or `map→odom`**.
`ros2 node info` shows only `/clock`.

→ `slam.launch.py` uses `LifecycleNode` with chained
`EmitEvent`/`OnStateTransition`: activate only after configure returns OK.

### 4. `docking_server` without `dock_plugins` takes down all of Nav2

It is part of the default Nav2 Jazzy list. Without configuration it fails to
configure and the lifecycle manager **aborts everything** — `map_server` and
`amcl` stay `active` and the rest stops at `inactive`. This demo has no dock;
the minimum is configured.

### 5. An empty YAML list breaks the launch

`docks: []` → Python tuple → the launch aborts. Omit the key.

### 6. CycloneDDS `autodetermine` picks the Docker bridge

On the Aquila, `ip -br addr` shows `ethernet0` **and** `br-a00dfb945795`
(172.18.0.1) UP at the same time — Toradex's own easy-pairing stack runs under
compose. `autodetermine` ranks interfaces and may pick the bridge. CycloneDDS
then transmits on an address the host does not route, and **no log on either
side mentions an interface**.

→ `scripts/module.sh sync` pins the interface, detected from `MODULE_IP`. Do not
swap it for `autodetermine` "to simplify", and do not pin `ethernet0` by hand —
a different board or a move to `wlan0` leaves the name obsolete and failing
silently.

### 7. colcon's `--packages-skip` is not `--packages-ignore`

`colcon`'s dependency graph does not distinguish `exec_depend` from
`build_depend`. `--packages-skip gz_quadruped_hardware` keeps the package in the
graph without building it, and `demo_simulation` (which declares it as an
`exec_depend`) fails asking for `install/gz_quadruped_hardware/.../package.sh`.
The real damage is `demo_bringup` coming afterwards and turning into "not
processed" — and that is where `nav.launch.py` and `perception.launch.py` live,
the entrypoints of the module's two services.

→ Use `--packages-ignore`. The error message does not mention the difference.

### 8. Two publishers on `/demo/cmd_vel` produce no error at all

`nav` on the module is Nav2, and Nav2 publishes `/demo/cmd_vel`. If the host's
simulation is running on the same `ROS_DOMAIN_ID`, the simulated robot starts
receiving commands from two sources. The topic is valid, both publishers are
healthy, DDS does exactly what it was told — and the robot moves on its own. A
gait trial in progress is **corrupted, not interrupted**.

→ `scripts/module.sh up` detects an active simulation on the host and refuses.
The ways out are in the refusal message.

### 9. `docker compose exec` does not run the image ENTRYPOINT

```
$ docker compose exec -T tools bash -lc 'which ros2'
                      # (nothing)
```

It is `entrypoint.sh` that `source`s the underlay and the `/ws/install` overlay,
and `exec` does not run it. `bash -lc` does not save you: the `ros` image does
not put the setup into `.bashrc`.

What makes this poisonous is the usual combination:

```bash
ros2 topic list 2>/dev/null | grep /demo/ || echo "(nenhum topico visivel)"
```

`2>/dev/null` swallows `command not found`, and `||` prints the **same line** a
real discovery failure would print. This has already made two verification steps
report a DDS problem that did not exist.

→ Always use `docker compose exec <svc> /usr/local/bin/entrypoint.sh <comando>`.
And do not use `2>/dev/null` in a diagnostic command. With `exec -d`, confirm
afterwards that the node came up — `-d` hides every error.

### 10. `ROS_NAMESPACE` does not work in ROS 2

```
$ ... -e ROS_NAMESPACE=/demo ... printenv ROS_NAMESPACE
/demo
$ ros2 run demo_tutorials heartbeat_publisher              → /system/heartbeat
$ ros2 run ... --ros-args -r __ns:=/demo                   → /demo/system/heartbeat
```

The variable **is** in the process environment. ROS 2 ignores it — it is a
leftover from ROS 1.

→ Use `--ros-args -r __ns:=<ns>`. Note that `scripts/env.sh` exports
`ROS_NAMESPACE=/demo` as if it worked; that line has no effect.

### 11. Half of a DDS configuration fails exactly like a firewall

With the module configured correctly (multicast off, peers) and a publisher
**demonstrably running** on it, the host saw nothing. Two causes at the same
time:

| Direction | Why it failed |
| --- | --- |
| host → module | The CycloneDDS default announces over **multicast**; the module has `AllowMulticast=false` and never listens. |
| module → host | The module sends unicast SPDP to the host's RTPS ports, but a default participant **does not pin a deterministic port** — it uses an ephemeral one and relies on multicast to be found. There is no port to aim at. |

Both sides need **matched** config: multicast off, `ParticipantIndex=auto`, and
the other address as a `<Peer>`. Configuring only one side produces exactly the
symptom of a firewall blocking.

→ `scripts/module.sh sync` renders both: `module.xml` (goes to the module) and
`docker/cyclonedds/host.rendered.xml` (stays on the host, gitignored). Whoever
publishes on the host must point `CYCLONEDDS_URI` at the rendered file.
`scripts/run_quadruped_sim.sh` mounts and selects that file when it exists and
prints the interface and peer in use.

### 12. Killing `ros2 launch` leaves the nodes alive, and the next Nav2 dies blaming DDS

Symptom: you restart Nav2 and **all** the nodes die on startup, each one with

```
[rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
  what():  failed to initialize rcl node: error not set, at ./src/rcl/node.c:252
```

and the `lifecycle_manager` stays forever at `Waiting for service
controller_server/get_state...`.

The message blames CycloneDDS. The culprit is the **previous** run. `ros2 launch`
is only the parent: a `kill` on it **orphans the child nodes**, which stay alive
holding a domain participant index. Measured on 21/08/2026: 31 orphans
accumulated over 5 launch generations, 14 domain failures on the following
startup. An isolated node still created a domain without error, which makes it
look like DDS is fine — and it is; what ran out was the index space.

How to confirm, before touching any DDS configuration:

```bash
ps -eo pid,etimes,comm --no-headers | grep -E \
  'odom_tf|controller_serv|bt_navigator|behavior_server|route_server'
```

If PIDs show up with an `etimes` larger than your current session, they are
orphans.

How to clean up. Use `pkill -x`, which matches the process **name**, and not
`pkill -f`:

```bash
for n in odom_tf cmd_vel_si_to_s velocity_smooth waypoint_follow \
         behavior_server smoother_server route_server opennav_docking \
         controller_serv planner_server bt_navigator collision_monit \
         lifecycle_manag; do pkill -9 -x "$n"; done
```

Two details that cost time on their own:

- **`pkill -f` matches the caller's own command line.** `pkill -f nav2` typed in
  a shell whose command contains `nav2` kills the shell. It happened twice here,
  and the symptom is the command "failing" without printing anything. `-x` does
  not have that problem. If you need `-f`, write the pattern with a character
  class: `pkill -f '[n]av2'`.
- **The names in `-x` are truncated at 15 characters**, which is the `comm`
  limit on Linux: it is `collision_monit`, not `collision_monitor`.

### 13. In HIL mode the robot gets slow and it is not the Aquila's fault

The symptom: in `hil` the robot navigates much more slowly than in `learn`, and
the temptation is to say the AM69 is weak. Measured on 21/08/2026, **it is not**.

What crosses the Wi-Fi is what weighs. The largest item, measured on the wire:

```
/demo/camera/image_raw: 640x480 rgb8, 921600 bytes/frame, 10,1 Hz
                        -> 74,2 Mbit/s
```

It is **raw** `sensor_msgs/Image`, uncompressed, with reliable QoS — every loss
becomes a retransmission, and retransmission becomes backpressure on the
publisher **inside the simulator**. That is why the effect shows up in the
robot's speed and not as a network error. Nothing in any log names the camera.

Bringing up only `nav` on the module, without `perception`, nobody subscribes to
the camera, CycloneDDS does not transmit it, and average speed rises 2,2×
(0,0197 → 0,0427 m/s). With the whole module stopped, 0,0725 m/s.

How to check before blaming the hardware:

```bash
# actual topic bandwidth, on the host side
python3 - <<'EOF'
import time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
rclpy.init(); n = Node('cam'); got = []
n.create_subscription(Image, '/demo/camera/image_raw',
                      lambda m: got.append(len(m.data)), 10)
t = time.monotonic()
while time.monotonic() - t < 10: rclpy.spin_once(n, timeout_sec=0.1)
print('%.1f Mbit/s' % (sum(got) * 8 / 1e6 / (time.monotonic() - t)))
EOF
```

And do **not** use `ros2 topic hz` for this: in this DDS configuration it comes
back without printing anything, on any topic, which looks like a dead topic.

On 21/08/2026, a probe and the `detection_stub` received no frames and this was
attributed to Wi-Fi. The 24/08 Ethernet HIL located the cause: the bridge
publishes the 921600-byte frame as `RELIABLE`, but the stub was asking for
`BEST_EFFORT`; discovery happened and every fragmented frame was lost. The stub
now asks for `RELIABLE` and receives ~10 Hz. So the old trial does not prove
that Wi-Fi was incapable of carrying the camera.

### Switching the host to Ethernet

The module has two Ethernet ports on the same `/24`, but the kernel routes that
network's peers through `ethernet0` (metric 101 against 102). Use `ethernet0`
explicitly; pinning DDS to `ethernet1` while the route leaves through
`ethernet0` creates divergent bind and send. Do not assume the mDNS hostname
identifies a port.

1. Before plugging the cable in, confirm that the host PHY advertises gigabit:

   ```bash
   ethtool enp0s31f6 | sed -n '/Supported link modes:/,/Advertised/p'
   ```

   `1000baseT/Full` must appear. If the I219 advertises only 10baseT after a
   resume from suspend, reload `e1000e`; if it still does not appear, power the
   host off for real — a warm reboot preserves that PHY state.

2. Connect the host and the module's `ethernet0` to the same switch/router,
   never point to point for this gate. Confirm link and route:

   ```bash
   ip -br addr show enp0s31f6
   ethtool enp0s31f6 | grep -E 'Speed:|Duplex:|Link detected:'
   ip route get <IP_ETHERNET0_AQUILA>   # must say "dev enp0s31f6"
   ```

   The gate requires `Speed: 1000Mb/s`, `Duplex: Full`, `Link detected: yes` and
   a wired `src` on the same subnet. If the route uses Wi-Fi, do not measure.

3. Pass the addresses explicitly. A variable in the environment beats
   `docker/.env`, including when it is empty; this is covered by a test:

   ```bash
   MODULE_HOST=<IP_ETHERNET0_AQUILA> MODULE_IP=<IP_ETHERNET0_AQUILA> \
     HOST_IP=<IP_ETHERNET_DO_HOST> ./scripts/module.sh sync
   ```

4. **Re-render the DDS configuration.** This is the step that gets forgotten:

   ```bash
   ./scripts/module.sh sync
   ```

   `sync` derives the interface from `ip route get`, so it then pins Ethernet on
   both sides. Without this, `host.rendered.xml` keeps pinning Wi-Fi, CycloneDDS
   transmits on an address the module does not answer, and the symptom is
   identical to a firewall. `run_quadruped_sim.sh` warns when the two diverge —
   read the `DDS:` line at startup.

5. Restart the simulator and the module's containers and require the
   instrument's three stages:

   ```bash
   MODULE_HOST=<IP_ETHERNET0_AQUILA> MODULE_IP=<IP_ETHERNET0_AQUILA> \
     HOST_IP=<IP_ETHERNET_DO_HOST> ./scripts/module.sh up
   MODULE_HOST=<IP_ETHERNET0_AQUILA> MODULE_IP=<IP_ETHERNET0_AQUILA> \
     HOST_IP=<IP_ETHERNET_DO_HOST> ./scripts/module.sh verify
   ```

   `verify` now creates the host subscriber before the remote heartbeat, waits
   for unicast discovery with a bounded deadline, and returns failure if UDP,
   the topic contract or the heartbeat do not pass.

6. Run three trials without changing load, image or parameters:

   ```bash
   python3 scripts/nav_trial.py docs/results/ml35-f5-ethernet0-run1.csv \
     --seconds 420 --goal-timeout 200 --sim-log <LOG_DA_SIMULACAO>
   # restart the plant in the same state and repeat as run2 and run3
   ```

   Each CSV records `sim_s` and `wall_s`; the summary prints the RTF computed
   over the same interval. F5 only closes if all three runs complete at least
   one 8 m goal, without a fall and with comparable link/RTF.

If Ethernet is not possible, the other paths are lowering the rate or resolution
in `demo_simulation/urdf/go2_sim.urdf.xacro` (which changes what the demo shows)
or compressed `image_transport`. Details in `docs/results/ml35-hil-aquila.md`.

### Isolated exhibition switch (no router/DHCP)

An ordinary Ethernet switch is layer 2 only: it forwards the frames, but does
not assign addresses or provide a default route. So the demonstration works
without a router as long as the host and the Aquila get static IPs on the same
subnet. The exhibition bench configuration is:

| Equipment | Interface | Address | Gateway |
| --- | --- | --- | --- |
| Host | `enp0s31f6` | `<HOST_IP>/24` | none |
| Aquila | port `ethernet0` connected to the switch | `<MODULE_IP>/24` | none |

On the host, the persistent NetworkManager profile must keep Wi-Fi as the
default route and use only the Ethernet's connected route. This prevents turning
off the external router from taking down host--Aquila communication:

```bash
nmcli connection modify ethernet \
  ipv4.method manual ipv4.addresses <HOST_IP>/24 \
  ipv4.gateway '' ipv4.never-default yes ipv4.route-metric 700 \
  ipv4.routes <MODULE_IP>/32 ipv6.method disabled
nmcli connection up ethernet ifname enp0s31f6
```

Before bringing the demo up, this is the minimum gate. `ip route get` must
mention `enp0s31f6`; if `ip neigh` stays `FAILED`, the Aquila is not on the
switch in that subnet (cable, port, power or module IP), and synchronizing DDS
will not fix the absence of L2 connectivity.

```bash
ip -br -4 addr show enp0s31f6
ip route get <MODULE_IP>
ping -c 2 -I enp0s31f6 <MODULE_IP>
```

When the Aquila answers, render the unicast peers again and validate the
contract, without depending on multicast or on the router:

```bash
./scripts/module.sh sync
./scripts/module.sh up
./scripts/module.sh verify
```

### 14. `use_composition` without a container is a 100% silent failure

`navigation_launch.py` with composition uses `LoadComposableNodes` to load the
servers inside `/nav2_container`, but **does not create** that container — the
one that creates it upstream is `bringup_launch.py`, which this project does not
include.

Turning `use_composition: 'True'` on without creating the container loads the
nodes into a container nobody created: **nothing comes up and nothing prints an
error**. The launch log ends at `wait_for_clock` and `odom_tf` and stops there.
The readable symptom is "Nav2 did not activate", which does not point at this
parameter.

The container is created in `nav_quadruped.launch.py`, in the `nav2_container`
block, with the name matching the default of the vendored launch's
`container_name` argument. If either of the two names changes, it fails this way
again.

### 15. `down` without `--profile` leaves `nav` alive with the old image

`nav` and `perception` are behind `profiles: ["learn"]` in `compose.host.yml`.
Compose **ignores services with a profile** in any command that does not declare
the profile, and that includes `down`:

```bash
docker compose -f compose.host.yml down --remove-orphans   # DOES NOT stop nav or perception
docker compose -f compose.host.yml --profile learn down     # stops them
```

`down` without the profile prints a list of removed containers that **looks
complete** — it lists what it removed, never what it skipped. The following
`up -d sim` does not recreate `nav` either, because as far as Compose is
concerned it is already in the desired state. Result: `nav` survives image
rebuilds indefinitely running the code from when it came up.

The symptom is the worst possible one, because there is no symptom:
`docker compose ps` says `running`, the topics exist, and the behavior is that
of an old version of the code. Measured on 25/08/2026 — a `nav` from 10 hours
earlier survived two build cycles and kept constructing the `ReroutingService`
that the freshly compiled fix no longer constructed, which made the fix look
like it had not worked.

How to check, when a result does not match the code:

```bash
# ID of the image running in the container vs. ID of the current tag
docker inspect docker-nav-1 --format '{{.Image}}'
docker images --no-trunc --format '{{.Repository}}:{{.Tag}} {{.ID}}' | grep nav:dev
```

Different IDs = stale container. It is the same family as trap 4: there the
`/ws/src` baked into the image was old, here the whole image is old.

---

### 16. The multi-arch builder cannot see local images, and the build "does not exist"

`CLAUDE.md` says to create the multi-arch builder once per host:

```bash
docker buildx create --use --name multiarch
```

`--use` leaves that builder **active for everything**, and it uses the
`docker-container` driver. That driver has its own image store and **does not
read Docker's local store**. Consequence: any image in this project whose parent
is another local image stops building, because the builder tries to *pull* the
parent from Docker Hub:

```text
failed to solve: local/demo-aquila-base:dev: failed to resolve source metadata
for docker.io/local/demo-aquila-base:dev: pull access denied, repository does
not exist or may require authorization
```

The message talks about authorization and a nonexistent repository, so it
**sends the reader looking for a credential, a VPN or a registry** — and the
problem is none of the three. The image exists, on the machine, built minutes
earlier.

`base` is deceptive because **it keeps working**: its parent is
`ros:jazzy-ros-base`, which really is pullable. Only `sim`, `nav`, `perception`,
`tools`, `viz`, `cockpit` and `hmi` break — which makes it look like a defect in
those images' Dockerfiles.

How to check and how to fix:

```bash
docker buildx ls          # the builder marked with * is active; the docker-container driver is the problem

docker buildx use default                                   # the `docker` driver reads the local store
docker compose -f compose.host.yml build base               # IN SERIES: the children
docker compose -f compose.host.yml build sim                # require the parent to be built already
docker buildx use armbuilder                                # restore it; multi-arch depends on it
```

Two notes that save time:

- **`docker compose ... --builder default` does not exist** in this version of
  Compose; it answers `unknown flag: --builder`. Switching the active builder is
  the way.
- **Building `base sim perception` in a single command fails because of a
  race**, not because of the builder: Compose fires all three in parallel and
  the children do not find the parent that is still being built. Always in
  series.

The **native build on the module** goes through none of this: it runs
`docker build` on the Aquila itself, with the local daemon.

---

### 17. `use_sim_time: true` costs CPU even in a node that never looks at the clock

The symptom: the module ends up under high load, `collision_monitor` rejects the
LiDAR cloud saying the source is old, and the robot stops. You look at `top`,
see Nav2 at the top and conclude that Nav2 is expensive. **It may not be it.**

`use_sim_time: true` is not a statement of intent. It is **rclpy** that creates
one `/clock` subscription per node, regardless of whether the node's code calls
the clock. In the Go2 world Gazebo publishes `/clock` at ~870 Hz, because the
gait's physics step is 1 ms. Every node that subscribes pays per message,
whether it uses it or not.

Measured on 26/08/2026 on the AM69, stack up and **with no active goal**: 367%
of 800%, of which 111% in three Python republishers that do not have a single
call to `get_clock()`.

How to check, instead of assuming. `top` shows the process; what you want is the
thread, and in a composed container the thread name is what gives up the
culprit:

```bash
# on the module. Sum utime+stime per thread across the container's entire cgroup.
CID=$(docker inspect -f '{{.Id}}' demo-nav-1)
for p in $(cat /sys/fs/cgroup/system.slice/docker-$CID.scope/cgroup.procs); do
  for t in /proc/$p/task/*; do
    read -r comm < $t/comm
    set -- $(cat $t/stat); echo "${t##*/} $comm $(( $14 + $15 ))"
  done
done
```

Run it twice with a ~20 s gap and take the difference: `(ticks2 - ticks1) /
100 / dt * 100` is the percentage of one core. A `component_container` spread
across ~30 threads at ~7% each is **not** algorithmic cost — it is message
delivery.

And confirm from the outside, without inferring:

```bash
ros2 topic info /clock -v | grep 'Node name'   # who actually subscribes
```

**Before removing `use_sim_time` from a node, verify that it does not read the
clock:**

```bash
grep -n 'get_clock' <arquivo_do_no>.py
```

If it does read it and you remove it, the node starts reading **wall time**
thinking it is reading simulated time. The stamps come out years into the
future, the costmap discards the reading with `message filter dropping message`,
and nothing names the cause. `demo_bringup/test/` `test_sim_time_scope.py` locks
this invariant in both directions.

**What this is NOT:** throttling `/clock` (`demo_simulation/clock_throttle.py`).
That lowers the rate for **everyone**, MPPI included, was tried on 21/08 and
**killed navigation** (0,0039 m/s against 0,0251). Here the rate does not change
for anyone; what changes is who subscribes.

### 18. `docker/.env` has an old bench address, and `ssh` hides it

The symptom: `scripts/module.sh sync` or `build` fails with

```
[module.sh] ERRO: nao identifiquei a interface do modulo que carrega <MODULE_IP>
```

while `ssh`, `verify` and `status` work normally.

The cause: precedence is environment → `docker/.env` → defaults. The bench
`.env` carries a `MODULE_IP` from another network, and it **beats the defaults**.
`ssh` does not notice because it uses `MODULE_HOST` (the mDNS name), not the IP —
only the commands that need to match the IP to an interface break.

Immediate way out:

```bash
MODULE_IP=<ip real> HOST_IP=<ip real> ./scripts/module.sh sync
```

Permanent way out: fix the `.env`. Check the real value with
`getent hosts <MODULE_HOST>` and `ip route get <ip do módulo>`.

### 19. `/demo/sim/reset` on a walking quadruped knocked it over

`/demo/sim/reset` (`std_srvs/Trigger`, served by `sim_control_relay`) teleports
the robot back to the scenario's spawn pose. Called directly through
`ros2 service call`, without the cockpit, on a quadruped that is walking: until
26/08/2026 it could collapse (height dropped from 0,337 m to 0,162 m in 1 s) or
drag itself around spinning for dozens of meters chasing the orientation from
before the reset — both defects are silent, nothing in a log or in the service
response gave them away.

The cause and the fix are detailed in cockpit trap 9,
[here](#9-reset-knocked-over-the-walking-quadruped) (`SetEntityPose` preserves
velocity; `StateTrotting` re-anchors its posture only once). The reset now stops
the gait before teleporting and resumes it afterwards — because of that, on a
quadruped the call takes a few seconds longer than on a diff-drive before it
answers. If the `Trigger` returns `success=True` but the message mentions
`NAO foi reancorado`, the robot got stuck standing without accepting commands:
call `ros2 service call /demo/gait/resume std_srvs/srv/Trigger` manually. That
service only exists on the quadruped plant — on the diff-drive the reset message
says `sem /demo/gait/hold (planta sem gait)`, and that is not a failure.

Evidence: `docs/results/cockpit-reset-nao-destrutivo.md` §3.1.

## 10. The Aquila AM69 module

Everything here is `arm64` on Torizon OS, and **nothing graphical** (rule 1: the
AM69 exposes only OpenGL ES 3.2 and Vulkan 1.2, so Gazebo and RViz2 stay on the
x86 host).

The interface is `scripts/module.sh`. It resolves both addresses instead of
guessing: `MODULE_IP` by name resolution, `HOST_IP` by the **route to the
module** — taking the first address of the first UP interface would pick
`docker0` or `tailscale0` on the workstation and produce a peer nobody can
reach.

```bash
scripts/module.sh inventory   # OS, Docker, disk, links — before anything else
scripts/module.sh sync        # sources + rendered config to ~/demo on the module
scripts/module.sh build       # builds the arm64 images ON the module
scripts/module.sh verify      # 3 stages: UDP, module sees host, host receives module
scripts/module.sh up          # nav + perception (refuses if sim is active on the host)
scripts/module.sh shell       # shell in the tools container
```

### Why the build runs on the module and not under QEMU

`CLAUDE.md` documents `docker buildx --platform linux/arm64` from the host, and
that path is the right one once there is a registry and a multi-arch manifest.
For bring-up it is not:

- QEMU arm64 may not even be enabled on the workstation (`binfmt_misc` with no
  aarch64 handler, `docker buildx ls` without `linux/arm64` in the platforms);
- the module has 8 × Cortex-A72 and 31 GiB idle, and compiles `ros2_control` and
  `unitree_guide_controller` in minutes, not hours;
- **the workstation is where the gait trials run.** They measure *when* the
  robot falls. A QEMU build saturates the CPU and corrupts the measurement
  instead of merely delaying it.

What is lost: a local arm64-only image, without a multi-arch manifest. And
neither of the two paths measures performance (rule 5).

### What goes to the module and what does not

`sync` sends `ros2_ws/src`, `docker/entrypoint.sh`, `compose.module.yml` and
only the Dockerfiles for `base`, `nav`, `perception` and `tools`. **`sim/` and
`viz/` are not sent** — they are OGRE 2. Their absence on the module is part of
the guard, together with the rule-1 check that `build` runs on the finished
images (looking for `ogre|gz-rendering|gz-sim|gz-gui|rviz`, and not a generic
`gz`: `gz-cmake/math/tools/utils-vendor` come in through `sdformat`, are pure
CPU and violate nothing).

### DDS configuration, rendered and not committed

No address goes into git. `sync` renders `docker/cyclonedds/module.xml` — pins
the interface, injects `<Peer address="${HOST_IP}"/>` — and writes it to
`~/demo/cyclonedds/module.xml`, validating the XML at the end. The `127.0.0.1`
peer is still there and is *load-bearing*: with `AllowMulticast=false`, `nav` and
`perception` on the module cannot see each other without it.

If `ros2 topic list` comes back empty on the module, check in this order:

```bash
ssh torizon@<módulo> 'grep -E "<Peer |<NetworkInterface " ~/demo/cyclonedds/module.xml'
ssh torizon@<módulo> 'cat ~/demo/.env'          # is ROS_DOMAIN_ID the same on both sides?
scripts/module.sh verify                         # stage 1 distinguishes firewall from DDS
```

### What is still missing for full `hil` mode

- The functional gate passes: TF closes, arm64 Nav2 and perception come up on
  the Aquila and the contract crosses the boundary.
- The stability gate still depends on the physical gigabit link and the three
  420/200 s runs described above. No number over Wi-Fi closes F5.
- `odom_tf` still republishes ground truth from the simulation; therefore HIL
  validates neither leg-based localization nor a physical Go2.

---

---

# Part II: Web cockpit

How to run it, what each region of the screen shows, what the buttons do, and
what is not possible through them.

This document is **operational**. The bench evidence (measurements, captures,
failures found) is in [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md)
and [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md); the design
decisions and the phases are in [`ml35/plano-cockpit-web.md`](ml35/plano-cockpit-web.md).

> **Nothing here was executed on the Aquila AM69.** Everything below is `learn`
> mode, with all containers on the x86 workstation. The kiosk on the module is
> F2 of the plan and is still open.

---

## 1. What the cockpit is

A web page that shows the robot and lets you command it, without RViz and
without the Gazebo window. It replaced four attempts at **embedding X11
windows** into a single application — all of them failed, and their checkpoint
([`results/cockpit-standalone-parcial.md`](results/cockpit-standalone-parcial.md))
is marked as superseded. **Do not resume that path.**

The axis now is: render **from ROS 2 topics**. That changes what is possible.
RViz and Gazebo are OGRE 2 and need desktop OpenGL, so they could never go to
the Aquila (rule 1 of `CLAUDE.md` — the AM69 only exposes OpenGL ES 3.2 and
Vulkan 1.2). A browser drawing `nav_msgs/OccupancyGrid` on a `<canvas>` can. That
is why **this cockpit becomes the module's HMI in M3**, and the previous attempt
would not.

Two pieces:

| Service | What it is | Where it runs today | Where it runs in M3 |
| --- | --- | --- | --- |
| `cockpit` | `rosbridge_server` + `web_video_server` | host | Aquila |
| `hmi` | nginx serving the `hmi/` bundle | host | Aquila |

The bundle is HTML/CSS/ES modules **with no build step** — no npm, no bundler,
no `node_modules`. Editing a file and reloading the page is the whole cycle.
That is a design decision (Decision 5 of the plan): the npm chain is precisely
what must not travel to the module.

---

## 2. Running it

### Prerequisites

Once per machine:

```bash
cd docker
cp .env.example .env      # edit MAZE_MODELS if you will use the maze
```

### Bringing it up

```bash
cd docker
docker compose -f compose.host.yml --profile learn up -d --build
```

This brings up five services: `sim` (Gazebo + robot + bridges), `nav` (Nav2),
`perception` (detection stub), `cockpit` (rosbridge + video) and `hmi` (nginx).

**Open <http://localhost:8081>.**

Nav2 takes ~30 s to become ready. The page can be opened before that: the panels
come up empty and fill in on their own, and each one's freshness dot says
whether that topic has arrived yet. This is on purpose — a cockpit that hangs
until everything is ready does not show what is missing.

### Ports

All containers use `network_mode: host`, so **nothing here is a port mapping**:
the numbers below open directly on the workstation. If they collide with
something you already run (8080 is popular), change them in `docker/.env`.

| Variable | Default | Serves |
| --- | --- | --- |
| `COCKPIT_HMI_PORT` | 8081 | the page — **this is the one you open** |
| `COCKPIT_ROSBRIDGE_PORT` | 9090 | WebSocket the browser consumes |
| `COCKPIT_VIDEO_PORT` | 8080 | camera MJPEG |

### Tearing it down

```bash
docker compose -f compose.host.yml --profile learn down
```

### Editing the bundle

A change in `hmi/` (HTML, CSS, JS) does not need `colcon`, but does need an
image rebuild, because nginx serves a copy:

```bash
cd <raiz do repo>
DOCKER_BUILDKIT=0 docker build -f docker/hmi/Dockerfile -t local/demo-aquila-hmi:dev .
cd docker && docker compose -f compose.host.yml up -d --no-build --force-recreate hmi
```

Before reloading, run the bundle's tests — they need neither ROS nor a
container:

```bash
cd hmi && node --test "test/**/*.test.js"
```

The **quotes around the glob are mandatory**: without them the shell expands it,
`node --test` receives a directory, resolves to nothing and exits with code 0 —
green without having run a single test.

---

## 3. The screen

Five regions, in the layout of [`ml35/cockpit-division-view.png`](ml35/cockpit-division-view.png):

```
+---------------------------+--------+
|          SCENE            |  NAV   |
+---------------------------+--------+
|          LOGS             | CAMERA |
+---------------------------+--------+
|             CONTROL BAR            |
+------------------------------------+
```

| Region | Source | Answers which question |
| --- | --- | --- |
| **Scene** | `/demo/cockpit/scene_{iso,top}/image_raw` | did the robot move? |
| **Navigation** | `/global_costmap/costmap`, `/plan`, `/demo/scan`, TF | does it know where to go? |
| **Logs** | `/rosout` + telemetry from `/demo/cmd_vel` and `/demo/odom` | what is it trying to do? |
| **Camera** | `/demo/camera/image_raw` | what is it seeing? |
| **Bar** | WebSocket state | is the cockpit still talking to the robot? |

### The Scene panel is the most important and the least obvious

They are **two static world cameras** — not robot cameras — switched by the
`iso` / `topo` buttons in the header. They exist because they answer "did the
robot move?" without depending on any node of the stack: if Nav2, TF and
odometry all lie together, this image stays honest. It is the screen's
independent witness.

### The freshness dot

Each panel has a small dot in the corner of its header, with three states that
are **not the same thing**:

| Color | State | Means |
| --- | --- | --- |
| gray | `never` | that topic has never produced anything since the page opened |
| green | `live` | a sample arrived within that panel's window |
| orange | `stale` | it arrived earlier and **stopped** |

`stale` is what catches a dead simulator, and that is why it cannot look like
`never`.

### Colors

The palette is the Toradex identity: white background, blue `#00508c`, green
`#96c837`, orange `#ff5a00`. The **failure red (`#c0261b`) is deliberately not a
brand color**: "stalled for too long" and "dead" have to look like different
things from across the room, and using orange for both would make the worst case
disappear inside the common case.

On the map, the same logic: blue is the global plan, green is the robot, orange
is an obstacle, and the goal is violet — a color that is **not** a brand color,
precisely so it cannot be confused with a state.

Canvas color does not exist in JavaScript. `hmi/js/panels/palette.js` reads the
`--map-*` tokens from `hmi/css/tokens.css` once, at mount time. Changing theme
means touching one file.

---

## 4. The controls

### Clicking on the map sends a goal

Click on the green panel. A `NavigateToPose` goes to Nav2, and the HUD at the
bottom of the panel starts showing the state (`navegando · 5,04 m restantes`),
including the recovery count. A **cancelar meta** button appears in the header
while a goal is active.

A click too close to the robot (< 0,25 m) is treated as a mistake and ignored.

**During an autonomous search the click sends nothing** — see the next section.

### Autonomous search: start and cancel

Two buttons in the green panel's header, `iniciar busca` and `cancelar busca`.
They call `/demo/exploration/{start,cancel}` (`std_srvs/Trigger`), served by
`maze_explorer`, which runs **alongside Nav2** — on the Aquila, in `hil` mode.

What it does with no help at all: it starts without a prior map, extracts
frontiers from `slam_toolbox`'s live map, navigates to the best one, and when
perception recognizes the exit's magenta panel, it cancels the frontier and
approaches in 0,5 m steps. It does **not** know the maze: there is no waypoint,
no exit coordinate, and there is a test making sure the code mentions none.

The HUD then shows the search state in place of the goal state:

| State | Means |
|---|---|
| `waiting_map` | waiting for map, TF and the Nav2 servers |
| `selecting` | evaluating candidates with the `ExplorationGrid` planner |
| `navigating` | heading to a frontier |
| `homing_exit` | seeing the marker and approaching it |
| `completed` | reached the marker |
| `failed` | total deadline (600 s) or a declared failure |
| `cancelled` | the operator stopped it |

Along with it go the elapsed time, how many frontiers exist, `saída detectada`
when perception is seeing the panel, and the last failure message.

Three things that look like details and are not:

- **The click on the map is disabled while the search runs.** Two goal sources
  on the same `navigate_to_pose` preempt each other with no error at all in the
  log — it is the same class of failure as the two publishers on
  `/demo/cmd_vel`.
- **`iniciar busca` refuses the second call** while one is in progress, and the
  refusal shows up in the HUD.
- **`reiniciar nav` cancels the search first** before cycling the stack. Without
  that the explorer would send a new goal in the middle of the reset cycle.

### `SAÍDA CONFIRMADA` does not come from the explorer

The label only appears when `/demo/maze/escaped` is `true`, and what publishes
that is `maze_escape_validator`, **on the simulation side**, looking at ground
truth odometry: it requires crossing the opening *and* the whole robot being
outside.

The explorer does not subscribe to that topic. It is the demonstration's
arbiter, not an input to it — `state: completed` only says the robot got close
to the panel, which is not the same as having exited.

### Simulation: ▶ ⏸ ⟲

In the bar. They call `/demo/sim/{play,pause,reset}` (`std_srvs/Trigger`).

The **reset requires two clicks**: the first arms the button (it turns into
`confirmar` in orange for 4 s), the second executes. It returns the robot to its
initial pose. It does **not** clear the costmap — what clears the costmap is
[restarting navigation](#restarting-navigation), a separate button, on purpose:
putting the robot back without discarding what Nav2 already knows about the
world is what allows the two in sequence without losing work.

In the quadruped scenario the click takes a few seconds longer than in the
diff-drive, and that is expected: the robot is **stopped** before being
repositioned and only walks again afterwards — see
[trap 9](#9-reset-knocked-over-the-walking-quadruped).

The label next to it (`rodando` / `pausado` / `sem simulador`) is **not the echo
of the last click**: it comes from `/clock`. An echo would lie in every case that
matters — dead `sim` container, expired call, a pause done in the Gazebo GUI, a
world reset by somebody else. If the button says one thing and the label says
another, **the label is right**.

The difference between `pausado` and `sem simulador` is the one that costs:
samples arriving with the same simulated time is a pause; samples **stopping** is
absence.

### Scene camera: the pad over the image

Bottom-right corner of the blue panel. Three rows:

```
 ↺  ▲  ▼  ↻      rotate and tilt
 ◀  △  ▽  ▶      move
 +  −  recenter   zoom in / zoom out / return to the initial framing
 follow robot     enable/disable following (applies to BOTH views)
```

Holding a button repeats it. The commands act on **the camera that is on
screen** — switching between `iso` and `topo` switches the target with it.
`seguir robô` is the exception: it applies to both at once, because what it
changes is the **orbit target**, and there is no version of that which makes
sense for a single camera.

The browser publishes **deltas** on `/demo/cockpit/scene/cmd_view`
(`geometry_msgs/TwistStamped`, with `header.frame_id` choosing the camera). What
keeps the orbit, saturates the limits and writes the pose into Gazebo is the
`scene_view_controller` node. That is why reloading the page does **not** disturb
the framing, and two open cockpits do not fight over the pose.

### Following the robot

On by default. Both cameras follow `/demo/odom` at 10 Hz, and what moves is only
the orbit's **target** — azimuth, elevation and distance stay where they were.
Practical consequence: the measured warehouse framing (the `(-3, +3, 2,4)`
diagonal on the iso view, the 6 m height on the top view) keeps holding while the
robot walks, instead of the robot leaving the frame in fifteen seconds.

The move buttons (`◀ △ ▽ ▶`) stay useful with following on: with it on, the pan
becomes an **offset relative to the robot**, saturated at 15 m, and not a fixed
world point. It serves for looking at the robot from the side, or a little ahead
of it, without losing the follow. `recentrar` zeroes that offset along with the
rest.

The button's state comes from `/demo/cockpit/scene/following`
(`std_msgs/Bool`, latched), published by the node, and **not from the click
itself** — the same reasoning as the simulation label, and for the same three
cases: F5, a second cockpit open, and someone who turned following off through
`ros2 service call`.

Turning it off (`follow:=false` in the launch, or the button) gives back the
wide view of the scenario, which is what you want to inspect the whole world or
to compare with an earlier framing.

### Restarting navigation

**reiniciar nav**, in the green panel's header. It calls `/demo/nav/reset`
(`std_srvs/Trigger`), served by `nav_control_relay` **inside the container that
runs Nav2** — in `hil` that is the Aquila.

Like the simulation reset, it **requires two clicks** (the first arms it for
4 s). Not for symmetry: it discards the goal in progress, and one accidental
click in the middle of a demo costs the demo.

The sequence, in order, and every step exists for a reason:

1. `CancelGoal` on `/navigate_to_pose` with `goal_info` zeroed — cancels **all**
   goals, including the one the cockpit does not know exists (sent by
   `patrol_commander`, or by another tab);
2. `ClearEntireCostmap` on the global and the local one, **with the servers
   still active** — a deactivated node does not answer services, so clearing
   after pausing would clear nothing and raise no error;
3. `PAUSE` on `lifecycle_manager_navigation`;
4. `RESUME`.

Measured at 6,6 s on the host. The interrupted goal ends `CANCELED`, the servers
come back `active [3]`, and a new goal is accepted right afterwards.

The panel's HUD shows `reiniciando…` during the sequence and `reiniciado` at the
end. No timeout on the browser side: the one with the timeout is the node (60 s
per transition), and closing the tab in the middle does **not** interrupt the
reset.

**Why not `RESET` + `STARTUP`**, which is the obvious path: it takes the
container down. See trap 8.

Localization is **not** touched. `lifecycle_manager_localization` is left out of
the sequence on purpose: on the static map path, recycling AMCL throws the pose
away and the robot "gets lost" in a reset that was only meant to discard the
goal.

### Manual control: deliberately disabled

The bar's arrows and E-STOP are drawn, keyboard reachable, and **inert**, with
the reason in the `title`. Today Nav2 is the only publisher on `/demo/cmd_vel`;
a teleop button publishing there would give two unarbitrated writers on the same
topic — last writer wins, and neither of them knows it lost. It closes in **F4**,
with `twist_mux`, not with a mux hand-written in the browser.

---

## 5. What you CANNOT do

### Run the simulation on the Aquila

It is not an implementation limitation, it is rule 1 of `CLAUDE.md`: Gazebo is
OGRE 2 and needs desktop OpenGL; the AM69 exposes only OpenGL ES 3.2 and
Vulkan 1.2. **No amount of UI code changes that.**

What does exist is **controlling the simulation from the cockpit**. In M3, with
the cockpit served by the Aquila, the click leaves the module and the service
call crosses the ROS graph — exactly as the Nav2 goal already crosses it today.
The simulator process stays on the x86 workstation.

If the distinction feels subtle at demo time: what is on the module's screen is
an **image** coming from the host, and a **button** that talks to the host.

### Call a service with a Gazebo type from the browser

Also not a temporary limitation. `rosbridge` assembles the request by importing
the interfaces package **inside its own container**, and the `cockpit` container
does not have `ros_gz_interfaces` — nor should it, because in M3 it runs on the
Aquila and in `deploy` mode there is no Gazebo at all. See trap 2 in
[section 9](#9-the-cockpit-traps-that-have-already-cost-time).

---

## 6. Switching scenario

The default world is the `nav2_minimal_tb4_sim` warehouse. For the 11,6 ×
11,6 m maze, two environment variables (in `docker/.env` or exported before the
`up`):

```bash
export MAZE_MODELS=/caminho/para/ros_maze_worlds/models

export SIM_ARGS="world:=/ws/install/demo_simulation/share/demo_simulation/worlds/quadruped_maze11.sdf \
  yaw:=1.5708 \
  scene_top_x:=-4.855 scene_top_y:=4.855 scene_top_z:=13.0 \
  scene_iso_x:=-13.0 scene_iso_y:=-3.0 scene_iso_z:=9.0 \
  scene_iso_pitch:=0.6717 scene_iso_yaw:=0.7676"

docker compose -f compose.host.yml up -d --force-recreate sim
```

What each group does:

- **`MAZE_MODELS`** — `sim` mounts that directory at `/maze/models` and points
  `GZ_SIM_RESOURCE_PATH` there. **Without it the world loads and the maze simply
  is not there** — with no error at all. The models come from
  `github.com/cafemesa/ros_maze_worlds` and no external asset is copied into the
  repository.
- **`yaw:=1.5708`** — the maze's starting corner has a wall at the default `+x`;
  without rotating, the robot spawns facing the wall.
- **`scene_*`** — framing of the two scene cameras. The values above were
  **measured**, not chosen: they are the ones that capture the whole maze
  (`scene_top`) and a legible diagonal (`scene_iso`).

`SIM_ARGS` is free-form and is passed straight to `sim.launch.py`. It is where
everything that belongs to the **scenario** and not to the **mode** lives. Empty
= warehouse.

### Why the scene cameras are models and are not in the worlds

Two reasons that only show up later:

1. the default world is third-party and **not editable** — and it is the one
   that comes up when nobody passes `world:=`;
2. hanging the camera off the vendored `go2_description` would break the
   **byte-for-byte guarantee** that underpins the licensing argument.

That is why they are spawnable models
(`demo_simulation/models/cockpit_scene_{iso,top}.sdf`), planted into any world by
`scene_cameras.launch.py`.

---

## 7. Tuning image quality

Current default: **1600 × 1200 at 10 Hz**, anti-aliasing 8, JPEG quality 95.

**The rate is 10 and not 15 because of measurement.** Both cameras render in the
same Gazebo process, and at this resolution the workstation does not deliver
15 Hz anyway. Measured on the bench, same world (`maze11`), 10 s window:

| `update_rate` | delivered on the topic | real-time factor |
| --- | --- | --- |
| 15 | 9,43 Hz | **0,59** |
| 10 | 9,77 Hz | **0,97** |

Asking for 15 did not yield a single extra frame and cost 40% of the
simulation's speed — which stretches every Nav2 goal in the same proportion.

**If you need to economize, change things in this order:**

1. **JPEG quality**, in `hmi/js/config.js` (`STREAM_QUALITY`). It degrades
   smoothly and changes nothing on the ROS side. It is the first place to touch
   if the bottleneck is **bandwidth** — `hil` mode, the stream crossing Ethernet
   to the Aquila.
2. **`update_rate`** in both SDFs, if the bottleneck is **rendering**.
3. **resolution**, last. It shifts the framing in pixels; and the **4:3 aspect
   ratio cannot change** — the `horizontal_fov` and the poses of the two cameras
   were measured at it, and going to 16:9 while keeping the hfov crops vertically
   and throws both scenes out of frame at once, with no error at all.

Any change to the SDFs requires rebuilding `base` **and** `sim` (the workspace is
compiled in `base`), and the Docker cache lies here — see trap 4.

### To measure the real-time factor

```bash
docker compose -f compose.host.yml exec -T sim bash -lc 'source /ws/install/setup.bash
  ros2 topic echo --once /clock; sleep 10; ros2 topic echo --once /clock'
```

Divide the advance in simulated time by the 10 s of wall time.

---

## 8. Cockpit diagnostics

The cockpit was built to answer "is this topic arriving?" on its own — that is
what the topic name printed in each header and the freshness dot are for. When
that is not enough:

```bash
cd docker

# Does the page exist?
curl -o /dev/null -w '%{http_code}\n' localhost:8081/

# Did rosbridge start?
docker compose -f compose.host.yml logs cockpit | grep -i rosbridge

# Are the scene cameras publishing?
docker compose -f compose.host.yml exec -T sim bash -lc \
  'source /ws/install/setup.bash; ros2 topic hz /demo/cockpit/scene_iso/image_raw'

# Does MJPEG respond?
curl -o /dev/null -w '%{http_code}\n' \
  'localhost:8080/snapshot?topic=/demo/cockpit/scene_iso/image_raw'

# Do the simulation services exist?
docker compose -f compose.host.yml exec -T sim bash -lc \
  'source /ws/install/setup.bash; ros2 service list | grep /demo/sim'
```

**The browser console is a first-class source here.** Service call failures show
up there (`[cockpit] falha ao pausar a simulação: ...`) and, in more detail, in
the `cockpit` container's log.

### Common cockpit symptoms

| Symptom | Likely cause |
| --- | --- |
| Green panel empty, HUD says `sem TF map→base` | trap 1 |
| Simulation button does nothing | trap 2 — look at the `cockpit` log |
| Blue panel black, gray dot | world without the cameras: was `sim` recreated without `SIM_ARGS`? |
| Maze does not appear, empty floor | `MAZE_MODELS` not exported |
| I edited the bundle and nothing changed | the `hmi` image was not rebuilt |
| I edited an SDF and nothing changed | trap 4 |
| Everything gray, `desconectado` badge | `cockpit` died, or port 9090 collided |

---

## 9. The cockpit traps that have already cost time

### 1. `/tf_static` arrives **only once**, and which message it is is luck

Symptom: the green panel stayed at `sem TF map→base` on roughly half of the page
loads.

`rosbridge` delivers **one** latched message per subscription. Measured across
three new, consecutive subscriptions: the first brought the robot's edges, the
second `map→odom`, the third `map→odom`. `queue_length: 16` changes nothing —
the loss is **above** the client's queue.

Fix, already in the code: `nav-panel.js` resubscribes to `/tf_static` every
1,5 s, at most 8 times, and stops for good as soon as `lookup('map','base')`
resolves. It converges in 2 to 4 rounds.

### 2. The browser cannot call a service with a Gazebo type

Symptom: the simulation buttons did absolutely nothing. The UI showed no error;
the cause only appeared in the `cockpit` container's log:

```
call_service InvalidModuleException: Unable to import ros_gz_interfaces.srv
from package ros_gz_interfaces. Caused by: No module named 'ros_gz_interfaces'
```

`rosbridge` assembles the request by importing the interfaces package **inside
its own container**, and `cockpit` does not have `ros_gz_interfaces`. And it must
not: in M3 it runs on the Aquila, and in `deploy` mode there is no Gazebo at all.

Fix: the **`sim_control_relay`** node (simulator side) exposes
`/demo/sim/{play,pause,reset}` as `std_srvs/Trigger` and translates to
`ControlWorld`. The browser boundary only speaks ROS core types. Guarded by
`tests/test_cockpit_web_contract.py::test_browser_never_speaks_gazebo_interfaces`.

### 3. The world's name is not the file's name

Gazebo's services live under `/world/<nome>/...`, and `<nome>` is the attribute
of the `<world>` element. `quadruped_maze11.sdf` declares
`<world name="quadruped_maze11">`, but the `nav2_minimal_tb4_sim` warehouse
declares `<world name='warehouse'>`. Guessing from the file name gets one case
right and the other wrong, **silently**: the bridge comes up, advertises the ROS
services, and every call times out on a gz service that does not exist.

That is why `sim_control.launch.py` parses the SDF, and a file without a
`<world>` takes the launch down naming which file it was.

### 4. The Docker cache lies about `COPY ros2_ws/src`

Symptom: you edit an SDF or a `.py` in the workspace, rebuild, and the change is
not in the container. The build reports `CACHED` for the `COPY` layer.

It happened three times in one session. **Workaround: run
`docker compose build base` twice** — the second one takes. And the workspace is
compiled in `base`, so any change in `ros2_ws/` requires `base` **and then**
`sim`:

```bash
export DOCKER_BUILDKIT=0
docker compose -f compose.host.yml build base
docker compose -f compose.host.yml build base   # yes, again
docker compose -f compose.host.yml build sim
```

Always confirm before concluding that the change did not work:

```bash
docker compose -f compose.host.yml exec -T sim \
  grep update_rate /ws/install/demo_simulation/share/demo_simulation/models/cockpit_scene_iso.sdf
```

**The worse variant is running a package's test suite inside the container.**
`/ws/src` is a copy in the image, not a bind mount of your working directory: an
image from before your edit runs the **old** tests and passes. A test you have
just written is simply not collected, and the output is green. It happened on
25/08: `32 passed` on the rebuilt image against `22 passed` on the previous one,
with the new file absent from the collection list. Check the count, or check the
collection:

```bash
docker compose -f compose.host.yml run --rm -T tools \
  bash -lc 'ls /ws/src/demo_simulation/test/'
```

### 5. Buildx cannot see local images

`docker compose build` with the `armbuilder` builder active fails with
`pull access denied ... local/demo-aquila-base:dev`. The multi-architecture
builder does not see the local daemon. For bench builds:

```bash
export DOCKER_BUILDKIT=0 BUILDX_BUILDER=default
```

### 6. `node --test` with an unquoted glob passes without running anything

```bash
node --test "test/**/*.test.js"    # correct
node --test test/**/*.test.js      # green without running a test
```

Without quotes the shell expands it, Node receives a directory, resolves to
nothing and exits with code 0.

### 7. Camera framing is measured, not guessed

The first attempt (iso at `-7,-7,5`) landed **inside** the warehouse's shelf
aisles — the robot became a white dot behind a shelf. The top view at 12 m hit a
roof beam exactly above the robot. Both current framings came out of attempts
measured against the scenario's real footprint.

When testing an orbit command from the command line, use
`ros2 topic pub -t 1 -w 1` and not `-r 3`: six seconds at 3 Hz apply ~18 steps of
0,35 rad ≈ 2π, the camera returns to its starting point, and it looks like
nothing happened.

### 8. `RESET` + `STARTUP` on Nav2 kills the container (segfault in `route_server`)

This was the natural design for "restart nav": Nav2's `lifecycle_manager` has
`RESET` (deactivates and unconfigures everything) and `STARTUP` (configures and
activates everything), and no other pair of transitions describes "restart the
stack" so well.

Measured on 24/08/2026, on the host, `learn` mode, the `nav2_container` dies:

```
[component_container_isolated-4] [INFO] [route_server]: Configuring Rerouting service operation.
[ERROR] [component_container_isolated-4]: process has died [exit code -11]
```

`-11` is `SIGSEGV`. Reproduced **twice** — with an active goal and without one.
It is not the "it happened once" this guide used to record: it is deterministic,
and it is on the `CONFIGURE` path, which is exactly what `STARTUP` does.

`route_server` is in the `lifecycle_nodes` list of the vendored
`navigation_launch.py` — which has to remain **identical to upstream** (see
`launch/nav2_vendored/README.md`), so removing it from the list is not an option.
This demo does not use routing, and it has no section in `nav2_params_go2.yaml`;
the hypothesis is that it reconfigures over state that does not survive
`CLEANUP`, but this was not confirmed in the Nav2 source. **Candidate for an
upstream issue.**

What `nav_control_relay` does instead — cancel, clear costmaps, `PAUSE`,
`RESUME` — never goes through `CONFIGURE`, and therefore never gets close to
this. If someone ever "simplifies" the sequence to `RESET`+`STARTUP`, the symptom
will be the `nav` container restarting and the cockpit losing the link in the
middle of the demo. There is a structural guard in `tests/` for exactly that.

#### It is not only on reset: it happens on a normal boot (25/08/2026)

The sentence above, "it is on the `CONFIGURE` path", was right and too narrow.
`route_server`'s `CONFIGURE` also happens during a **normal startup** of `nav`,
and there the same segfault appears — **intermittently**: the same
`docker compose up nav` came up one time and took the stack down the next.

The state it leaves behind is what makes this expensive:

```
$ docker compose ps
nav   running                       <- this is false

$ docker exec docker-nav-1 ps -eo comm
ros2                                <- only the parent
odom_tf
nav_control_rel
cmd_vel_si_to_s                     <- no Nav2 server
```

**All** the servers die at once, the container stays `running`, and from inside
it `get_node_names()` lists only the `sim` nodes. Whoever looks at Compose
concludes "Nav2 is up"; whoever looks at the robot concludes "navigation got
worse". It was half of the regression investigated in
`docs/results/ml35-regressao-navegacao.md`.

The fix lives in `nav2_params_go2.yaml`: `route_server.operations` lists only
`AdjustSpeedLimit`, so the plugin that blows up is never constructed. Overriding
that list forces you to also declare the **type** of every plugin in it
(`AdjustSpeedLimit.plugin`), otherwise startup is rejected with
`Can not get 'plugin' param value` — a loud failure, and in that respect better
than the segfault.

### 9. Reset knocked over the walking quadruped

The simulation reset teleports the robot back to its initial pose via
`SetEntityPose` (see trap 2 on why it is not a direct Gazebo call from the
browser). That fixes a worse defect — `reset.all` **deleted** the whole robot,
see `docs/results/cockpit-reset-nao-destrutivo.md` §1-2 — but on its own it was
not enough for the quadruped, and for two different reasons, discovered in
sequence:

1. **teleporting without re-anchoring the gait controller.** `StateTrotting`
   (the C++ gait controller) captures its posture reference only once, and a
   teleport changes the pose without going through that capture. Measured with
   no velocity command published for 26 s: even so the robot dragged itself
   0,87 m and rotated 135° on its own, chasing the pose from BEFORE the reset;
2. **teleporting without stopping.** `SetEntityPose` repositions the body and
   **preserves the velocity**. A walking robot, teleported, is released still
   traveling with its legs in swing — and falls. Measured with Nav2 actually
   driving during the reset: the robot's height dropped from 0,337 m to 0,162 m
   in 1 second.

The fix stops the robot BEFORE teleporting and re-anchors it AFTERWARDS — two
internal services (`/demo/gait/hold`, `/demo/gait/resume`), served by the same
node that already translated `/demo/cmd_vel` into the gait's axes. Neither of the
two is exposed to the cockpit; the operator only sees the effect, which is the
reset click taking ~2-7 s longer on the quadruped than on the diff-drive.
Verified even with the robot **fallen** (tipped over, stuck in the controller's
recovery mode — which does not get out of it on its own while upside down): the
reset puts it back on its feet.

Complete evidence: `docs/results/cockpit-reset-nao-destrutivo.md` §3.1.

---

## 10. How the cockpit is built

### Tree

```
hmi/
├── index.html            the five regions
├── css/
│   ├── tokens.css        palette, typography, motion — SINGLE color source
│   ├── layout.css        five-region grid
│   └── panels.css        panel chrome, bar, camera pad
├── img/                  Toradex and ROS brands (white PNG with alpha)
├── js/
│   ├── main.js           wiring only: resolves config, mounts panels, one timer
│   ├── config.js         endpoints, topics, MJPEG URL
│   ├── ros/              transport
│   │   ├── rosbridge-client.js   WebSocket, reconnection, actions, services
│   │   ├── png-decompress.js     compressed costmap (33x smaller than JSON)
│   │   ├── tf-tree.js            TF cache
│   │   └── freshness.js          green/orange/gray dots
│   ├── panels/           rendering
│   │   ├── stream-panel.js       image panels (scene and camera)
│   │   ├── nav-panel.js          map canvas, click-to-goal
│   │   ├── map-view.js           world <-> screen, cost LUT (pure, tested)
│   │   ├── palette.js            reads CSS --map-* tokens
│   │   ├── log-panel.js          /rosout + telemetry
│   │   ├── control-bar.js        link state
│   │   ├── sim-controls.js       play/pause/reset
│   │   ├── view-controls.js      camera pad
│   │   └── detection-overlay.js  NOT mounted today — see below
│   └── ...
└── test/                 node --test, without a browser
```

### On the ROS side

| File | Role | Runs on |
| --- | --- | --- |
| `demo_bringup/launch/cockpit.launch.py` | rosbridge + web_video_server | host today, Aquila in M3 |
| `demo_simulation/launch/scene_cameras.launch.py` | plants the two scene cameras | host |
| `demo_simulation/launch/sim_control.launch.py` | gz service bridge + façade | host |
| `demo_simulation/scene_view_controller.py` | orbit of the scene cameras | host |
| `demo_simulation/sim_control_relay.py` | `std_srvs` façade for play/pause/reset | host |
| `demo_navigation/launch/nav_control.launch.py` | brings up the Nav2 reset façade | host or Aquila |
| `demo_navigation/nav_control_relay.py` | `std_srvs` façade for restarting Nav2 | wherever Nav2 runs |

The last two lines are the only pair in this table that **runs on the module** in
`hil` mode: they live in the `nav` container, alongside the stack they restart.
The simulation façade is tied to the host because Gazebo is.

### A detail that looks like a bug and is not

The camera panel **does not draw the detection boxes**, and the absence is
deliberate. Today's `demo_perception` is a deterministic stub: it sweeps a
synthetic box across the image whether there is an object there or not. Over the
video that becomes a rectangle strolling from one side to the other — worse than
nothing in a demo, because the viewer reads it as a real detection.

**The detections are still published and still feed the costmap's
`perception_layer`.** The `CLAUDE.md` topic contract is intact; only the drawing
was removed. `detection-overlay.js` is still in the bundle, tested, ready to come
back when TIDL replaces the stub.

### Tests

```bash
cd hmi && node --test "test/**/*.test.js"    # 138 — bundle logic
cd .. && python3 -m pytest tests/ -q          # 36 — structural guards
```

The structural guards are static checks on committed files, not runtime tests.
They exist because every invariant they cover is cheap to break in a one-line
edit and expensive to discover — the placement ones only fail on the Aquila,
weeks later.

---

## 11. What is done and what is missing

### Done (24/08/2026, host only)

- **F1** — transport and skeleton: `cockpit` and `hmi` services, bundle, live
  camera, automatic reconnection. Evidence: [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md).
- **F3b** — scene panel (two switchable cameras) and navigation panel (costmap,
  plan, laser, footprint, click-to-goal). Gate met: clicked goal accepted and
  executed by Nav2.
- **Simulation control** from the cockpit: play, pause, reset.
- **Camera control**: rotate, tilt, move, zoom, recenter.
- **Toradex identity** and repalettizing the map for a light background.
- **Image quality**: 800×600@5 Hz → 1600×1200@10 Hz, JPEG 70 → 95.

Evidence for the set: [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md).

### UI adjustments (25/08/2026, host only)

Three bench requests, outside the phase numbering:

- **Toradex logo at double size** (34 → 68 px tall; the ROS one, 22 → 44). The
  bar's minimum height went from 48 to 80 px **through the same token**
  (`--bar-min-height` in `tokens.css`) — the bar has `overflow-x`, not `-y`, so
  the two measurements diverging would crop the logo without warning.
- **Following the robot** in both scene views. Verified on the host:
  `scene_top` at `(-1,552 ; 0,151 ; 6,0)` against the robot at
  `(-1,598 ; 0,130)` — following within ~5 cm with `z` preserved; `scene_iso` at
  `(-4,551 ; 3,15 ; 2,4)`, that is, the measured offset `(-3, +3, 2,4)` kept
  while it slides along with the robot. Turning it off froze the pose for 6 s;
  turning it back on recentered.
- **Restarting navigation** from the cockpit, in 6,6 s, with the interrupted
  goal ending `CANCELED` and the servers coming back `active [3]`.

None of this was seen in a browser with a screen capture: Chrome is not
installed on this machine and headless Firefox snap did not respond. What exists
is the page served with the correct HTML and CSS (`curl` 200, tokens and both
buttons present in what nginx delivers) plus verification from the ROS side.
**None of this ran on the Aquila AM69** (rule 7 of `CLAUDE.md`); the
measurements are all from the x86 host in `learn`.

### Missing

- **F4 — manual control.** `twist_mux` arbitrating against Nav2. It is the only
  region of the screen that still lies, and that is why the buttons are
  disabled.
- **F2 — kiosk on the module.** Chromium on the Aquila serving this same bundle.
  Three things only the module can answer:
  1. the bundle was verified in **Firefox**; the kiosk is Chromium, and the
     `<img>` cache caveat in `config.js` comes from the literature, not from
     measurement;
  2. the MJPEG at 1600×1200 crossing the Ethernet — none of that was measured
     there;
  3. in `deploy` mode there is no Gazebo: the simulation buttons need to
     disappear or say why they do not apply. **Not handled yet.**

### A known open item, unrelated to the cockpit

The `route_server` segfault on configure, now characterized and deterministic —
see [trap 8](#8-reset--startup-on-nav2-kills-the-container-segfault-in-route_server).
It is not a cockpit regression; it is the reason the navigation reset uses
`PAUSE`/`RESUME` instead of `RESET`/`STARTUP`.

---

---

## References

- `.ai/CLAUDE.md` — operational contract, inviolable rules, current phase
- `.ai/AGENTS.md` — implementation contract, milestones, Definition of Done
- `.ai/changelog.md` — decisions taken and why
- `CLAUDE.md` at the root — the inviolable rules, rule 1 above all
- README of each package in `ros2_ws/src/demo_*/`
- [`ml35/plano-cockpit-web.md`](ml35/plano-cockpit-web.md) — web cockpit decisions and phases
- [`ml35/estado-fases.md`](ml35/estado-fases.md) — ML3.5 state, read it first in a new session
- [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md), [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md) — cockpit bench evidence
