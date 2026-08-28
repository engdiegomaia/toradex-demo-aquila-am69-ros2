# Simulation sensors and the navigation chain

Analysis document for phase L3. Describes **exactly** which sensor data comes
out of Gazebo, where it flows through on its way to Nav2, and how the navigation
command travels back down to the robot's actuators.

Every number below was **measured** in simulation (Gazebo Harmonic 8.14.0,
world `nav2_minimal_tb4_sim/worlds/warehouse.sdf`), not copied from
documentation. Where a value could not be verified, it is marked as such.

> **Where this runs:** everything in this document runs on the **x86
> workstation**. Gazebo, `ros_gz_bridge` and RViz2 never run on the Aquila AM69
> module (rule 1 of `CLAUDE.md`: the AM69 GPU only exposes OpenGL ES 3.2 /
> Vulkan 1.2, and Gazebo is OGRE 2 / desktop OpenGL). In `target` mode, only
> Nav2, bringup, perception and `rosbridge_server` move to the module — the
> simulator stays on the host.

---

## 1. The robot model

The robot is the **upstream TurtleBot 4**, included directly from
`nav2_minimal_tb4_description` (maintained by the Nav2 team):

```
/opt/ros/jazzy/share/nav2_minimal_tb4_description/urdf/standard/turtlebot4.urdf.xacro
```

`demo_description/urdf/demo_robot.urdf.xacro` is now just a **thin wrapper**
around that file.

**Why this changed.** The previous version assembled the robot piece by piece
from the individual TB4 meshes, with offsets derived from bounding-box
measurements — hard to maintain and error-prone. The upstream model already
comes assembled.

### ⚠️ The robot with "separated parts" in RViz: the most likely cause

**Check this BEFORE touching the model.** Two model swaps and one physics fix
did not resolve this symptom, because the cause was in the RViz configuration —
not in the robot.

`nav2_bringup/rviz/nav2_default_view.rviz` (which `learn.launch.py` used) ships
with:

| Display | Upstream value | Effect |
| --- | --- | --- |
| `RobotModel` | `Enabled: false` | **the robot body is not drawn** |
| `TF` | `Enabled: true`, Show Axes + Show Names | **33 axis triads** drawn |

Result on screen: no robot body and 33 labelled axis markers floating (4 tower
columns, 4 weight blocks, 6 bumper zones, 6 OAK-D frames, IMU, wheels, caster)
exactly where the robot should be. That **looks identical** to a robot with its
parts separated.

Diagnosed by reading the display flags, not the URDF.

**Fix:** `demo_bringup/rviz/demo_view.rviz` — a copy of the Nav2 file with
`RobotModel` on and `TF` off. To debug frames, tick `TF` in the sidebar and
untick it afterwards.

```bash
# Confirm the flags of the file RViz actually loads
python3 -c "
import yaml
d=yaml.safe_load(open('install/demo_bringup/share/demo_bringup/rviz/demo_view.rviz'))
for x in d['Visualization Manager']['Displays']:
    if x.get('Class','').endswith(('RobotModel','TF')):
        print(x['Class'].split('/')[-1], x.get('Enabled'))
"   # RobotModel True / TF False
```

### ⚠️ Unwelded fixed joints (a distinct problem, also real)

Swapping the model did **not** resolve this symptom, and it is worth recording
why, because the previous explanation (wrong mesh offsets) was **incorrect**.

Upstream marks 22 fixed joints with:

```xml
<gazebo reference="..."><preserveFixedJoint>true</preserveFixedJoint></gazebo>
```

That tag tells Gazebo **not** to weld the joint's child into the parent's rigid
body. Without welding, the robot is born as **13 independent physics bodies**
(shell, 4 tower columns, sensor plate, lidar, camera, bumper, wheels, caster,
base), linked only by fixed-joint constraints. The solver does **not** keep them
rigid: under gravity and contact the parts settle and drift apart — the tower
tilts, the plate and lidar float above the columns.

**This is not a geometry bug.** The offsets and the per-visual `rpy` rotations of
the upstream model are correct. Welding or not welding does not change **where
the parts are drawn**, only whether physics can move them relative to one
another. That is why re-measuring offsets never fixed it — and why swapping the
robot model did not fix it either.

**Detail that makes this hard to attribute:** RViz2 **ignores** `<gazebo>` blocks
entirely, so it always drew the robot correctly assembled. The model looks
perfect in RViz and falls apart in Gazebo.

**Fix.** Xacro cannot remove a tag emitted by an `include`, so the removal
happens in the pipeline that feeds Gazebo:

```bash
weld_fixed_joints.py demo_robot.urdf.xacro robot_name:=demo_robot > robot.urdf
```

`demo_simulation/launch/simulation.launch.py` already does this. If you spawn
that xacro through any other path, run it through the script as well.

Two details about the script, both learned the hard way:
- It is **silent** on success. The launch `Command` substitution aborts the
  entire launch if the command writes **anything** to stderr
  (`executed command showed stderr output`) — a friendly progress message was
  bringing the simulation down. Use `--verbose` when running it by hand.
- It is called **instead of** `xacro`, not piped into it: `Command` passes the
  string through `shlex.split`, not through a shell, so a `|` would reach xacro
  as a literal argument.

Locked down by `test_weld_script_removes_every_preserve_fixed_joint`.

**Positive side effect:** we gained sensors that did not exist before — a stereo
RGBD camera (OAK-D Pro) and an IMU.

### Relevant TF tree

```
odom                              ← published by the Gazebo DiffDrive plugin
  └── base_link                   URDF ROOT; Nav2's robot_base_frame
        ├── base_footprint        IDENTITY transform (xyz 0 0 0, rpy 0 0 0)
        ├── shell_link
        │     ├── rplidar_link            z = +0.193 m
        │     └── oakd_camera_bracket
        │           └── oakd_link
        │                 └── oakd_rgb_camera_frame → …_optical_frame  z = +0.244 m
        ├── imu_link                      z = +0.084 m
        ├── left_wheel / right_wheel      (continuous joints)
        └── front_caster_link
```

**Careful — the tree is inverted relative to the old model:**

| | root | child |
|---|---|---|
| old model (ours) | `base_footprint` | `base_link` |
| current model (upstream) | `base_link` | `base_footprint` |

The upstream `DiffDrive` has its `child_frame_id` **hard-coded to `base_link`**,
defined in `icreate/create3.urdf.xacro`, and **there is no xacro argument** to
change it.

That is why Nav2 was pointed at `base_link` (`robot_base_frame: base_link` at
every point in `nav2_params.yaml`), instead of trying to force `base_footprint`.
This is safe because `base_footprint_joint` is an identity transform — the two
frames coincide numerically. There is a test
(`demo_description/test/test_urdf_parses.py::test_base_footprint_coincides_with_base_link`)
that **fails** if a future upstream update gives that joint a real offset and
breaks the equivalence.

> **Trap (cost real time).** The obvious attempt — redeclaring the
> `<gazebo><plugin ...DiffDrive>` block in our wrapper just to change the
> `child_frame_id` — **does not work**. Xacro **concatenates** `<gazebo>` blocks,
> it does not replace them. The result is a URDF with **two** DiffDrive plugins
> controlling the same two joints, both integrating odometry and publishing TF.
> Gazebo loads both without complaining. Check:
> ```bash
> xacro src/demo_description/urdf/demo_robot.urdf.xacro | grep -c diff-drive-system   # must be 1
> ```
> There is a test that locks this down: `test_exactly_one_of_each_gz_system_plugin`.

---

## 2. Sensor data received from the simulation

### 2.1 Complete table (measured values)

| Sensor | gz topic | ROS topic | ROS type | Rate | `frame_id` | Consumer |
|---|---|---|---|---|---|---|
| 2D lidar (RPLIDAR A1) | `/scan` | `/demo/scan` | `sensor_msgs/LaserScan` | 10 Hz | `rplidar_link` | Nav2 (both costmaps), AMCL, SLAM |
| RGB camera (OAK-D Pro) | `/rgbd_camera/image` | `/demo/camera/image_raw` | `sensor_msgs/Image` | 10 Hz | `oakd_rgb_camera_optical_frame` | `demo_perception` |
| Intrinsics | `/rgbd_camera/camera_info` | `/demo/camera/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | same | `demo_perception` |
| Depth | `/rgbd_camera/depth_image` | `/demo/camera/depth_image` | `sensor_msgs/Image` | 10 Hz | same | **none yet** |
| Point cloud | `/rgbd_camera/points` | `/demo/camera/points` | `sensor_msgs/PointCloud2` | 10 Hz | same | **none yet** |
| IMU | `/imu` | `/demo/imu` | `sensor_msgs/Imu` | 200 Hz | `imu_link` | **none yet** |
| Wheel odometry | `/odom` | `/demo/odom` | `nav_msgs/Odometry` | 30 Hz | `odom` → `base_link` | Nav2 (controller, bt_navigator) |
| Joint angles | `/joint_states` | `/joint_states` | `sensor_msgs/JointState` | 30 Hz | — | `robot_state_publisher` |
| Clock | `/clock` | `/clock` | `rosgraph_msgs/Clock` | — | — | **all** nodes |

### 2.2 Lidar — measured parameters

Verified with `gz topic -e -t /scan -n 1`:

```
frame_id   = rplidar_link
count      = 360          samples per scan
range_min  = 0.164 m
range_max  = 20.0 m
update     = 10 Hz
```

The **20 m** `range_max` matters: the old hand-built lidar declared 12 m, and
`nav2_params.yaml` was still configured for 12 m. See "Optimization 2".

### 2.3 IMU — depends on the world, not just the robot

The IMU sensor is declared in the URDF, but it **only publishes if the WORLD
loads the `gz-sim-imu-system` system**. The `warehouse.sdf` from
`nav2_minimal_tb4_sim` does load it (verified). A hand-written world may not, and
in that case the sensor stays **silent, with no error at all**.

```bash
gz topic -l | grep imu      # if empty, the world has no imu-system plugin
```

That is exactly how this document nearly recorded "IMU dead": a minimal test
world without the plugin. The sensor was correct the whole time.

### 2.4 How the data crosses the Gazebo → ROS boundary

Gazebo **does not speak ROS**. The translation is done by `ros_gz_bridge`
(`parameter_bridge`), configured in
`demo_simulation/config/bridge_warehouse.yaml`. Each line is a tuple
(gz topic, ROS topic, type, direction).

```
Gazebo (gz-transport, protobuf)        ros_gz_bridge         ROS 2 (DDS/CycloneDDS)
  /scan          gz.msgs.LaserScan   ───────────────►   /demo/scan     sensor_msgs/LaserScan
  /rgbd_camera/image  gz.msgs.Image  ───────────────►   /demo/camera/image_raw
  /imu           gz.msgs.IMU         ───────────────►   /demo/imu
  /odom          gz.msgs.Odometry    ───────────────►   /demo/odom
  /cmd_vel       gz.msgs.Twist       ◄───────────────   /demo/cmd_vel  geometry_msgs/Twist
```

**Naming trap (documented in the YAML itself).** All names on the gz side are
**literal and unscoped** (`/cmd_vel`, `/odom`, `/scan`). Gazebo *also* advertises
similar, model-scoped names (`/model/demo_robot/cmd_vel`) that **show up in
`gz topic -l` but have no publisher behind them at all**. Bridging those names
produces a robot that never moves and odometry that never publishes — **with no
error anywhere**. Always check with `gz topic -l` while the simulation is
running.

---

## 3. How navigation reaches the actuators

### 3.1 Complete chain

```
        ┌─ /demo/scan (10 Hz) ──► costmaps (obstacle_layer)
        │                          ├─ global_costmap  (1 Hz)  ──► planner_server
        │                          └─ local_costmap   (5 Hz)  ──► controller_server
        │
        ├─ /demo/scan ──────────► AMCL ──► TF: map → odom
        │
        ├─ /demo/odom (30 Hz) ──► controller_server (current velocity)
        │
        └─ /demo/perception/detection_cloud ──► perception_layer (both costmaps)

      goal (RViz2 / BT) ──► bt_navigator ──► planner_server (NavFn, 20 Hz)
                                                    │
                                                    ▼ nav_msgs/Path (global)
                                            controller_server (20 Hz)
                                                    │  MPPI: 2000 trajectories
                                                    ▼
                                          geometry_msgs/Twist
                                                    │
                                            /demo/cmd_vel
                                                    │  ros_gz_bridge (ROS→GZ)
                                                    ▼
                                            /cmd_vel  (Gazebo)
                                                    │
                                    gz-sim-diff-drive-system plugin
                                                    │  inverse kinematics
                                                    ▼
                            torque on left_wheel_joint / right_wheel_joint
                                                    │
                                    ┌───────────────┴────────────────┐
                                    ▼                                ▼
                        /odom (30 Hz) + TF odom→base_link      /joint_states (30 Hz)
                                                                      │
                                                          robot_state_publisher
                                                                      ▼
                                                            wheel TF (visual)
```

### 3.2 The single actuation point: `/demo/cmd_vel`

There are no joint controllers, no `ros2_control`, no `controller_manager`. All
actuation goes through **a single topic**: `/demo/cmd_vel`
(`geometry_msgs/Twist`).

The `gz-sim-diff-drive-system` plugin receives that `Twist` and solves the
differential inverse kinematics:

```
v_left  = (v_linear − ω · L/2) / r
v_right = (v_linear + ω · L/2) / r

where  L = wheel_separation = 0.233 m
       r = wheel_radius     = 0.03575 m
```

**`wheel_separation` and `wheel_radius` are the two values that can never
diverge from the physical model.** The plugin integrates them to produce
odometry; if they diverge from the meshes/collisions, the robot **visibly slides
while reporting a straight line** — and nothing raises an error. Inheriting these
values from upstream (instead of redeclaring them) keeps them locked to the
geometry by definition.

Verified at runtime: a `linear.x = 0.3` command for 2 s moved the robot
**0.6046 m** (expected 0.6 m) with odometry tracking it.

### 3.3 Limits imposed by the simulator

The upstream `DiffDrive` **saturates internally**:

| Limit | Value |
|---|---|
| `max_linear_velocity` | 0.5 m/s |
| `max_angular_velocity` | 2.0 rad/s |
| `max_linear_acceleration` | 2.0 m/s² |
| `max_angular_acceleration` | 3.0 rad/s² |

**Practical consequence:** configuring Nav2 with velocities above these values
makes the controller command speeds that the simulator **silently refuses to
reach**. The symptom looks like a controller tuning error — and it is not. The
limits in `nav2_params.yaml` were aligned to these values (`vx_max: 0.5`,
`wz_max: 1.9`, with margin on ω).

### 3.4 Who publishes each TF edge

Duplicating any of these produces a jittering robot, with no error message.

| Edge | Publisher |
|---|---|
| `map → odom` | AMCL (or `slam_toolbox` in SLAM) |
| `odom → base_link` | Gazebo DiffDrive plugin (via the `/tf` bridge) |
| `base_link → *` (sensors, wheels) | `robot_state_publisher` |

---

## 4. Optimizations

### Applied in this session

#### Optimization 1 — `robot_radius` 0.22 → 0.18 m

The real radius of the TB4 chassis is ~0.17 m (`shell_radius` 0.12 + bumper). The
0.22 value came from the old model's chassis and added ~5 cm of phantom width.
Effect: the robot **refuses gaps it physically fits through**, which in a
warehouse aisle shows up as a planner failure.

#### Optimization 2 — lidar range 12 → 20 m

The `obstacle_layer` had `raytrace_max_range: 12.0`, inherited from the old
lidar. The real sensor reaches **20 m** (measured). When the raytrace range is
smaller than the real range, the layer **stops clearing cells it never actually
observed**, leaving stale obstacles frozen in the costmap.

Also added `obstacle_min_range: 0.17` (above the sensor's `range_min` of 0.164):
closer returns are measurement artifacts, and marking them plants obstacles
**inside the robot's own footprint**, triggering spurious recovery behaviors.

`obstacle_max_range` (15 m) is deliberately **below** `raytrace_max_range`
(20 m): mark obstacles conservatively, clear free space generously.

#### Optimization 3 — inflation 0.55 → 0.35 m, `cost_scaling_factor` 3.0 → 5.0

0.55 m is **more than 3× the robot radius**. In the warehouse's narrow aisles, a
0.55 m "skirt" on each wall made the whole aisle nearly lethal: the global
planner routed around aisles the robot fits through, and the local one oscillated
inside them.

0.35 m covers the 0.18 m footprint with ~0.17 m of clearance. **Never reduce it
below `robot_radius`** — inflation smaller than the footprint allows paths whose
curves graze obstacles.

`cost_scaling_factor` 5.0 makes cost drop faster with distance, letting the
planner commit to the **center** of the free aisle.

Both costmaps were kept **identical** on these values: divergent inflation
between global and local is a classic cause of "the global plan goes where the
local one refuses to follow" — the robot stalls in the middle of the aisle.

#### Optimization 4 — DWB → MPPI

**Why.** DWB scores a fixed grid of constant-curvature arcs. On a differential
drive in a narrow aisle this produces two visible artifacts: it **oscillates
between neighboring samples** (the robot snakes down a straight aisle) and it
cannot represent a maneuver that requires a velocity sign reversal mid-trajectory,
so tight turns degenerate into stop-turn-go.

MPPI samples 2000 noisy trajectories per cycle and takes the cost-weighted
average, so the command is **continuous** instead of pinned to a grid sample. It
is Nav2's current recommendation for differential drive.

Configuration details:
- `motion_model: DiffDrive` — **not** `Omni`. This robot does not strafe;
  declaring `Omni` makes MPPI sample unexecutable lateral velocities and then
  chase a path it can never follow.
- `CostCritic` instead of `ObstaclesCritic`: it respects the inflation layer's
  gradient instead of applying its own footprint model, so the `inflation_radius`
  tuned above is in fact what guides the robot.
- The 8 critics were **verified** against `libmppi_critics.so`. A critic with a
  wrong name **is not loaded and raises no error** — it simply stops
  contributing.

> ⚠️ **CPU cost — only measurable on real hardware.** MPPI is significantly
> heavier than DWB. On the x86 host (`learn` mode) it is comfortable. On the
> Aquila AM69 in `target` mode this is the most expensive node in the stack, and
> **whether 20 Hz holds there cannot be answered from `learn` mode or from arm64
> emulation** (rule 5 of `CLAUDE.md`: emulation does not measure performance). It
> has to be measured on the real module. If it does not hold, reduce `batch_size`
> to 1000 **before** lowering `controller_frequency` — halving the batch costs
> less precision than halving the control rate.

#### Optimization 5 — expose depth, point cloud and IMU on the bridge

Bridged the new sensors to `/demo/camera/depth_image`, `/demo/camera/points` and
`/demo/imu`. **Not consumed yet** — but available and measurable, and ready for
the TIDL (MX-TIDL) work without changing the bridge.

> ⚠️ **Bandwidth:** `/demo/camera/points` is by far the heaviest topic
> (~640×480 XYZRGB at 10 Hz). In `target` mode that DDS traffic crosses the
> network to the module. Comment the entry out if the demo does not need 3D
> obstacles.

### Recommended, not yet applied

These were left out **deliberately**, because they change structure (not just
tuning) and deserve isolated validation:

#### A — IMU + odometry fusion via EKF (`robot_localization`)

**Gain.** Pure wheel odometry accumulates heading drift on **every** rotation
(wheel slip). The 200 Hz IMU corrects exactly that. It is the best
cost/benefit localization improvement available right now.

**Why it was not applied now.** An `ekf_node` becomes the owner of the
`odom → base_link` edge, which today belongs to the DiffDrive plugin. **Two
publishers on the same edge produce a jittering robot, with no error message.**
It requires:
1. creating `demo_navigation/config/ekf.yaml`;
2. **disabling** DiffDrive's TF publication (or removing `/tf` from the bridge);
3. revalidating the tree with `ros2 run tf2_tools view_frames`.

`robot_localization` is already installed (`ekf_node` verified).

#### B — 3D obstacles via `voxel_layer` on the point cloud

Today the robot only sees obstacles **in the lidar plane** (z ≈ 0.19 m). It is
blind to low pallets and to overhangs above the plane. The OAK-D cloud is already
on the bridge; what is missing is a `nav2_costmap_2d::VoxelLayer`. It costs CPU
and bandwidth — measure on the module.

#### C — `RotationShimController` wrapped around MPPI

Makes the robot **rotate in place** to align with the path before accelerating,
instead of pulling away in an arc. It considerably improves how the demo reads
visually at starts and on tight turns. Verified as installed
(`nav2_rotation_shim_controller::RotationShimController`).

#### D — Replace NavFn with Smac Planner (`SmacPlannerHybrid`)

NavFn produces grid paths with 45° corners. Smac Hybrid generates kinematically
feasible paths for differential drive. The gain is aesthetic and in smoothness;
the cost is more CPU in the global planner (which runs at 1 Hz, so the impact is
smaller than in the controller).

---

## 4.5 Goals in unknown space (not a tuning bug)

Symptom in the log, when clicking a goal in RViz:

```
[ERROR] [planner_server]: Failed to create a plan from potential when a legal
        potential was found. This shouldn't happen.
[WARN]  GridBased plugin failed to plan from (4.76, 0.41) to (11.01, 10.61):
        "Failed to create plan with tolerance of: 0.500000"
```

The "This shouldn't happen" message suggests an internal planner bug. **It is
not.**

The saved `warehouse.pgm` has **55% unknown cells**, because SLAM only mapped the
aisles the robot drove through:

| Class | Cells | % |
| --- | --- | --- |
| FREE | 98 863 | 43.6% |
| OCCUPIED | 3 393 | 1.5% |
| **UNKNOWN** | **124 319** | **54.9%** |

The goal `(11.01, 10.61)` is **entirely** in unknown space — value 205 in the PGM
across its whole 1 m neighborhood. The goal `(-4.84, 3.38)`, which worked, was in
free space (254).

`allow_unknown: true` **does not fix** this case: it allows crossing reachable
unknown cells, not carving a corridor to a goal surrounded by unknown.

**How to check a goal before clicking it:**

```bash
python3 - <<'EOF'
X, Y = 11.01, 10.61      # the goal you want to test
res, ox, oy = 0.05, -12.077, -12.215
p='install/demo_navigation/share/demo_navigation/maps/warehouse.pgm'
f=open(p,'rb'); f.readline(); l=f.readline()
while l.startswith(b'#'): l=f.readline()
w,h=map(int,l.split()); f.readline(); d=f.read(w*h)
cx, cy = int((X-ox)/res), int((Y-oy)/res)
v = d[(h-1-cy)*w + cx]
occ = (255-v)/255.0
print(f"pgm={v} ->", 'OCCUPIED' if occ>0.65 else ('FREE' if occ<0.196 else 'UNKNOWN'))
EOF
```

**Solutions, in order of effort:**
1. Click goals only in white (free) map area in RViz — gray is unknown.
2. Redo the map covering more area: run SLAM and drive the robot through the
   missing aisles before saving (`map_saver_cli`).
3. For the demo, define fixed, verified goals instead of free clicking.

This is **not** a regression from the section 4 optimizations — the first goal of
that same run completed with `Reached the goal!` / `Goal succeeded`.

## 5. How to verify all of this

```bash
cd ~/toradex/demo/aquila-am69-ros2/ros2_ws
source install/setup.bash          # mandatory in EVERY new terminal

# 1) The URDF expands, has 1 of each plugin, and the TF tree is as expected
python3 -m pytest src/demo_description/test/test_urdf_parses.py -q

# Count plugin ELEMENTS, not lines of text. Any grep here is useless: the xacro's
# own comments mention the plugin name and match the pattern. Parse the XML.
xacro src/demo_description/urdf/demo_robot.urdf.xacro | python3 -c "
import sys,xml.etree.ElementTree as ET
from collections import Counter
r=ET.fromstring(sys.stdin.read())
print(Counter(p.get('name') for g in r.findall('gazebo') for p in g.findall('plugin')))
"   # DiffDrive: 1, JointStatePublisher: 1

# 2) No preserved fixed joints — otherwise the robot falls apart in Gazebo
python3 src/demo_description/scripts/weld_fixed_joints.py --verbose \
  src/demo_description/urdf/demo_robot.urdf.xacro | grep -c preserveFixedJoint  # 0

# 2) Gazebo side: the literal names exist and have a publisher
gz topic -l | sort
gz topic -e -t /scan -n 1 | grep -E "frame|count|range_m"
gz topic -l | grep imu             # empty ⇒ the world has no gz-sim-imu-system

# 3) ROS side: the real rate of each sensor (not just "the topic exists")
ros2 topic hz /demo/scan           # ~10 Hz
ros2 topic hz /demo/odom           # ~30 Hz
ros2 topic hz /demo/imu            # ~200 Hz
ros2 topic hz /demo/camera/image_raw

# 4) TF tree with no duplicate edge and no orphan frame
ros2 run tf2_tools view_frames

# 5) End-to-end actuation
ros2 topic pub --once /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic echo /demo/odom --once  # the position must have changed
```

**General rule for this project:** a topic that exists in `ros2 topic list` does
**not** prove there is a publisher. Always use `ros2 topic hz` / `gz topic -i`.
Almost every silent failure documented here shows up as a topic that is present
and mute.
