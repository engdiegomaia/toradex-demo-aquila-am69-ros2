# Development guide — native x86 host

This document walks the team through the pre-hardware phases on Ubuntu 24.04 x86_64. Everything here runs **natively on the workstation, without Docker**. Containers enter at L4 (see `.ai/AGENTS.md` §9 ML4).

> **Why native first?** Container over an unfamiliar simulator makes it impossible to tell whether a broken `/camera/image_raw` is a Gazebo config bug, a bridge remap error, a DDS discovery issue, or a Docker networking problem. Learn each piece in one process tree first. This applies equally to ROS 2 and to Gazebo.

The pre-hardware sequence is:

| Sub-step | What you prove | Deliverable |
| --- | --- | --- |
| **L0a** | ROS 2 Jazzy is installed and CycloneDDS works | turtlesim driven by teleop, `ros2 doctor` clean |
| **L0b** | Gazebo Harmonic runs standalone with GPU rendering | `gz sim shapes.sdf` opens, `gz topic -l` populated |
| **L0c** | `ros_gz_bridge` connects the two worlds | `Twist` from `ros2 topic pub` moves something in Gazebo |
| **L1** | You can write a ROS 2 package | `demo_tutorials` heartbeat + service + launch, `colcon test` green |

L2 (URDF/TF2/RViz2), L3 (Gazebo + Nav2), L4 (containers + arm64 emulation) each get their own plan when reached.

---

## Prerequisites

- Ubuntu 24.04 LTS x86_64 (Noble). Do not use 22.04 — Jazzy Tier 1 platform is 24.04.
- Internet access to `packages.ros.org` and `packages.osrfoundation.org`.
- A GPU with a working OpenGL desktop driver (any modern Intel/AMD/NVIDIA is fine on the workstation). Verify: `glxinfo | grep "OpenGL version"` reports ≥ 3.3, preferably 4.x.
- `sudo` on the host.

Sanity-check the host once:

```bash
lsb_release -a                         # Ubuntu 24.04 LTS
uname -m                               # x86_64
glxinfo -B | grep -E "OpenGL version|OpenGL renderer"
```

If `glxinfo` is missing: `sudo apt install -y mesa-utils`.

---

## L0a — ROS 2 Jazzy + turtlesim

### Install

Follow the official Debian-package instructions (this is the canonical source; do not use unofficial mirrors):

```bash
# 1. Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Enable required repos
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# 3. Add the ROS 2 apt key and repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Install ROS 2 Jazzy Desktop + tools we need for the whole project
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-turtlesim \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete

# 5. Initialize rosdep once per host
sudo rosdep init || true                # ok if it says "already exists"
rosdep update
```

Reference: [ROS 2 Jazzy install (deb)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

### Environment contract

Every shell that touches this project should source two things:

```bash
source /opt/ros/jazzy/setup.bash        # ROS 2 itself
source scripts/env.sh                   # project-specific overrides (created in L1a)
```

Until `scripts/env.sh` exists, export the variables manually for L0 exercises:

```bash
export ROS_DOMAIN_ID=69
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
# Do NOT export ROS_NAMESPACE=/demo for turtlesim exercises — turtlesim publishes /turtle1/*
# and remapping it into /demo confuses beginners. Set the namespace only once you're
# running demo_tutorials nodes (L1).
```

### Verify

```bash
ros2 doctor                              # no critical warnings; RMW = rmw_cyclonedds_cpp
ros2 pkg list | grep turtlesim           # turtlesim is present

# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 run turtlesim turtle_teleop_key     # arrow keys move the turtle

# Terminal 3 — inspect the graph
ros2 topic list                          # /turtle1/cmd_vel, /turtle1/pose, /turtle1/color_sensor
ros2 topic echo /turtle1/pose
ros2 topic hz /turtle1/cmd_vel           # publish rate while you hold an arrow key
ros2 node list                           # /turtlesim, /teleop_turtle
ros2 node info /turtlesim
```

### Exit criterion for L0a

- Turtle moves under teleop.
- `ros2 doctor` clean.
- `ros2 topic echo /turtle1/pose` streams data.
- You can articulate the difference between a **node**, a **topic**, a **service**, and a **parameter** without looking it up.

### Useful references — L0a

- [ROS 2 Concepts](https://docs.ros.org/en/jazzy/Concepts.html) — nodes, topics, services, parameters, actions
- [CLI tools tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html) — walk every one of these once
- [Turtlesim intro tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [CycloneDDS ROS 2 RMW](https://github.com/ros2/rmw_cyclonedds) — why we pin it (see `.ai/CLAUDE.md` rule 2)

---

## L0b — Gazebo Harmonic standalone

Gazebo Harmonic is the **LTS release officially paired with ROS 2 Jazzy**. Do not install Fortress (older) or Ionic (newer) — the bridge package versions won't line up.

### Install

```bash
# 1. Add the OSRF apt repo
sudo apt install -y curl lsb-release gnupg
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 2. Install Gazebo Harmonic
sudo apt update
sudo apt install -y gz-harmonic
```

Reference: [Gazebo Harmonic install on Ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu/).

### Verify

Standalone Gazebo has its own transport (`gz-transport`), completely independent of ROS 2 / DDS. Learn the two CLIs — `gz sim`, `gz topic`, `gz service`, `gz model` — before mixing them.

```bash
# 1. Open a demo world
gz sim shapes.sdf                        # GUI opens; press the play (▶) button

# 2. In another terminal, inspect Gazebo's own transport
gz topic -l                              # /clock, /stats, /world/shapes/... etc.
gz topic -e -t /clock                    # streaming simulation time
gz service -l                            # world control, entity spawn, ...
gz model --list                          # entities in the running world

# 3. An empty world, and check that GPU rendering is real
gz sim empty.sdf
# In the GUI: Window ▸ System Info should show your GPU (not "llvmpipe" — that would
# be software rendering). If it says llvmpipe, install the vendor GL driver and retry.
```

### Verify Ogre 2 + OpenGL desktop, on purpose

This is the moment to internalize why Gazebo will **never** run on the Aquila:

```bash
glxinfo -B | grep -E "OpenGL version|OpenGL renderer|OpenGL core profile version"
# Workstation: something like OpenGL core profile version 4.6 (Intel/AMD/NVIDIA)
# Aquila BXS-4-64 would report OpenGL ES 3.2 only — no desktop OpenGL. Gazebo's
# default Ogre 2 renderer requires desktop GL ≥ 3.3. Hence rule 1.
```

Reference: [.ai/demo-ros2-aquila-am69.md §6.1 — why the simulator does not run on the module](../.ai/demo-ros2-aquila-am69.md).

### Exit criterion for L0b

- `gz sim shapes.sdf` opens with hardware rendering (not `llvmpipe`).
- `gz topic -l` and `gz topic -e -t /clock` both work.
- `gz service -l` lists services; you can pause/resume the world via CLI.
- You can articulate: Gazebo's transport is **not** ROS 2. The two systems only meet at `ros_gz_bridge`.

### Useful references — L0b

- [Gazebo Sim tutorials index](https://gazebosim.org/docs/harmonic/tutorials/) — start with "Understanding the GUI" and "SDF world basics"
- [SDFormat 1.10 spec](http://sdformat.org/spec?ver=1.10) — the XML you'll be writing in L3
- [gz-transport CLI](https://gazebosim.org/api/transport/13/index.html) — `gz topic`, `gz service`
- [Gazebo Harmonic release notes](https://gazebosim.org/docs/harmonic/releases/) — LTS support window

---

## L0c — `ros_gz_bridge` loop

Now connect the two worlds. Still no `demo_*` code — this is bridge fluency only. When L3 comes, the bridge is already understood.

### Install

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-image
```

`ros-jazzy-ros-gz` is the metapackage; the others are what you'll actually invoke by name.

Reference: [ros_gz repo (Jazzy branch)](https://github.com/gazebosim/ros_gz/tree/jazzy).

### Verify: bridge `/clock` from Gazebo → ROS 2

The simplest bridge. Prove data can cross before doing anything fancier.

```bash
# Terminal 1 — Gazebo
gz sim -r empty.sdf                        # -r starts unpaused

# Terminal 2 — bridge just the clock topic (Gazebo → ROS 2, one direction)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=69
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Terminal 3 — verify ROS 2 sees Gazebo's clock
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=69
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list                            # /clock present
ros2 topic echo /clock                     # streaming sim time
ros2 topic hz /clock                       # non-zero rate
```

**Bridge notation:** `<topic>@<ros_type>[<gz_type>` = **Gazebo → ROS 2**. Reverse arrow `]` = **ROS 2 → Gazebo**. Double arrow `@` = bidirectional. See the [ros_gz_bridge README](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md) — this notation is the single most common source of confusion in bridge configs.

### Verify: bridge `cmd_vel` ROS 2 → Gazebo

Spawn a mobile robot demo and drive it from ROS 2. Gazebo Harmonic ships with example worlds that already publish/subscribe the right topics — no SDF authoring yet.

```bash
# Terminal 1
gz sim -r diff_drive.sdf                   # ships with gz-harmonic; two robots side by side

# Terminal 2 — bridge cmd_vel (ROS 2 → Gazebo) plus /clock (Gazebo → ROS 2)
ros2 run ros_gz_bridge parameter_bridge \
  /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Terminal 3 — drive the blue robot forward
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.2}}"
# The blue robot moves in Gazebo. This proves the ROS 2 → Gazebo path end-to-end.
```

If `diff_drive.sdf` isn't on your Gazebo resource path, find one that is:

```bash
find /usr/share/gz -name "*.sdf" 2>/dev/null | grep -i drive
# or use any world at https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds
```

### Verify: image topic through the bridge

Cameras are the topic type most likely to bite later (large payloads, QoS, transport). Bridge one now.

```bash
# Use a world with a camera sensor, or add one to empty.sdf
gz sim -r sensors_demo.sdf                 # if available; otherwise camera_sensor.sdf

# Bridge the image (Gazebo → ROS 2). For image_transport-compatible republishing,
# ros_gz_image gives you an image_bridge helper:
ros2 run ros_gz_image image_bridge /camera

# Inspect on the ROS 2 side
ros2 topic hz /camera                       # expected framerate
ros2 topic info /camera                     # sensor_msgs/msg/Image
# Optional: visualize
ros2 run rqt_image_view rqt_image_view /camera
```

### Exit criterion for L0c

- `/clock` streams from Gazebo into ROS 2 via `parameter_bridge`.
- `ros2 topic pub` on a `Twist` moves a robot in Gazebo.
- A camera image is visible in `rqt_image_view` through the bridge.
- You can read `<topic>@<ros_type>[<gz_type>` at a glance and say which direction it flows.

### Useful references — L0c

- [ros_gz_bridge notation and examples](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md)
- [ros_gz_image bridge](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_image/README.md)
- [Bridge YAML config format](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md#example-4-using-yaml-file-to-configure-the-bridge) — you'll switch from CLI to YAML by L3
- [Gazebo Sim `diff_drive` example world](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive.sdf)

---

## L1 — Writing `demo_tutorials`

Everything from L1 onward lives in the repo. Once L0a/b/c are all green, proceed with the L1 tasks tracked in the task list:

- **L1a** — create `scripts/env.sh` with the project env contract
- **L1b** — scaffold `ros2_ws/src/demo_tutorials/` (`ament_python`)
- **L1c** — heartbeat publisher + subscriber on `/demo/system/heartbeat`
- **L1d** — `AddTwoInts` service example
- **L1e** — `heartbeat.launch.py` and `turtlesim_demo.launch.py`
- **L1f** — unit test + `ament_flake8` + `ament_pep257`
- **L1g** — final walkthrough section in this doc

**L1 exit exercise (single command satisfies ML1):**

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch demo_tutorials heartbeat.launch.py rate_hz:=2.0
# Subscriber logs receiving /demo/system/heartbeat at ~2 Hz with no counter gaps.
```

Plus: `colcon test` green, `ros2 param set` retunes the rate live, `ros2 service call` returns the sum. Detailed steps in the tracked tasks.

### Useful references — L1

- [Creating a Python package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Writing a publisher/subscriber (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Writing a service and client (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Using parameters in a class (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [Creating a launch file](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [`ament_python` package layout](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Python-Documentation.html) — the required-files list from `.ai/AGENTS.md` §5.1 comes from here

---

## Troubleshooting

### `ros2 topic list` shows the topic but no messages arrive

The default RMW has a known inter-process discovery failure. Confirm:

```bash
env | grep -E "^(ROS_|RMW_)"
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp must be set in *every* shell.
```

If any terminal is missing it, that terminal will not talk to the others. This is exactly why we bake the variable into the base image at L4 (rule 2 in `.ai/CLAUDE.md`).

### Two shells see different topic graphs

Check `ROS_DOMAIN_ID` — different domain IDs = different graphs, silently.

```bash
env | grep ROS_DOMAIN_ID     # must be 69 in every terminal that participates
```

### Gazebo starts with slow / choppy rendering

Almost always software rendering. Check:

```bash
glxinfo -B | grep "OpenGL renderer"
# If it says "llvmpipe" or "swrast" you're on software GL. Install the vendor driver.
```

### `gz sim` opens but has no models loaded

Resource path issue:

```bash
# See where Gazebo is looking for resources
echo $GZ_SIM_RESOURCE_PATH
# Or list built-in worlds
find /usr/share/gz -name "*.sdf" | head
```

### `ros_gz_bridge` reports "No message type conversion" or similar

Type mismatch in the bridge notation. Common mistakes:

- Wrong ROS type: `geometry_msgs/msg/Twist`, not `geometry_msgs/Twist`
- Wrong Gazebo type: `gz.msgs.Twist`, not `ignition.msgs.Twist` (Harmonic uses `gz.msgs.*`; that's a Fortress→Harmonic rename)
- Wrong arrow: `[` and `]` control direction. Bidirectional is `@`.

The [supported type table](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md#mapping-between-ros-and-gazebo-interfaces) is authoritative.

---

## Cross-references

- Project rules and non-negotiables — [`CLAUDE.md`](../CLAUDE.md) and [`.ai/CLAUDE.md`](../.ai/CLAUDE.md)
- Milestone specification — [`.ai/AGENTS.md`](../.ai/AGENTS.md) §9 (ML1 acceptance is the L1 exit criterion)
- Architecture rationale (why Gazebo stays on x86) — [`.ai/demo-ros2-aquila-am69.md`](../.ai/demo-ros2-aquila-am69.md) §6.1
