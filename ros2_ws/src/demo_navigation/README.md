# demo_navigation

Nav2 configuration, static map, and navigation launch. Delivers the navigation
half of ML3.

**Runs on:** Aquila AM69 (arm64) in `target` mode, x86 host in `learn` mode.
Architecture-neutral — this is the half of the demo that migrates to the module.
RViz is deliberately absent from `navigation.launch.py`: it is an OGRE 2
application and stays on the host (`CLAUDE.md` rule 1). `demo_bringup` starts it
separately in `learn` mode.

## Setup

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

## The map is already generated

`maps/warehouse.yaml` + `maps/warehouse.pgm` are committed (477 × 475 cells at
5 cm, origin `[-12.077, -12.215, 0]`). You only need the procedure below if the
world changes.

## Regenerating the map (only if the world changes)

Three terminals:

```bash
ros2 launch demo_simulation simulation.launch.py   # 1
ros2 launch demo_navigation slam.launch.py         # 2
ros2 launch demo_simulation teleop.launch.py       # 3 — drive the whole warehouse
```

Then save into the **source tree**, not `install/` (a symlink farm wiped by a
clean rebuild):

```bash
ros2 run nav2_map_server map_saver_cli -f \
    ros2_ws/src/demo_navigation/maps/warehouse
```

Commit both `warehouse.yaml` and `warehouse.pgm`. The module runs AMCL over the
saved map and never runs SLAM.

### Two things that will waste your afternoon

**slam_toolbox is a lifecycle node.** It starts in `unconfigured` and stays
there. The process runs and logs `Node using stack size 40000000`, then does
nothing: no scan subscription, no map, no `map → odom`, and no error.
`slam.launch.py` drives the `configure` → `activate` transitions with chained
lifecycle events. Check with `ros2 lifecycle get /slam_toolbox` — it must say
`active`. If you ever see it stuck in `unconfigured`, that is this bug.

**The scan topic is set by remap, not by parameter.** slam_toolbox reads
`scan_topic` while declaring parameters, and setting it from an external params
file does not reliably take effect — the node comes up subscribed to `/clock`
and nothing else. `slam.launch.py` remaps `/scan → /demo/scan` instead, which
rclcpp applies before the subscription is created. Verify with
`ros2 node info /slam_toolbox`; `/demo/scan` must appear under Subscribers.

## Parameters

Everything lives in `config/nav2_params.yaml` — never in code, per project
convention. Derived from the Nav2 Jazzy defaults, changed only where this robot
differs:

- topics namespaced under `/demo` (`/demo/scan`, `/demo/odom`, `/demo/cmd_vel`)
- `robot_base_frame: base_footprint`
- `robot_radius: 0.25` — circumscribed radius of the 0.40 × 0.30 chassis
- velocity limits matched to the robot: `max_vel_x: 0.5`, `max_vel_theta: 1.5`.
  The Nav2 stock 2.5 m/s is meaningless for a 0.35 m wheel separation.
- `amcl.initial_pose` must agree with the spawn pose in
  `simulation.launch.py`, or the particle cloud starts in the wrong place and
  the first goal fails.

## The perception costmap layer

`CLAUDE.md` requires detections to steer the robot, not just decorate the HMI.
Both costmaps therefore carry a `perception_layer`:

```
local:  [obstacle_layer, perception_layer, inflation_layer]
global: [static_layer, obstacle_layer, perception_layer, inflation_layer]
```

It is a stock `nav2_costmap_2d::ObstacleLayer` reading `PointCloud2` from
`/demo/perception/detection_cloud`, which `demo_perception`'s adapter produces
from `Detection2DArray`. Placed before `inflation_layer` so detections inflate
like any other obstacle. Rationale for the adapter over a C++ plugin is in
`demo_perception/README.md`.

Two settings are deliberate, and both follow from the stub having no depth:

| Setting | Value | Why |
| --- | --- | --- |
| `clearing` | `false` | The assumed-range projection is too coarse to trust for clearing; letting it clear would erase real lidar obstacles. |
| `observation_persistence` | `1.0` | Detections expire after a second instead of leaving phantom obstacles trailing the robot. |

This wiring stays in place when TIDL replaces the stub — that is the point.

## Running

```bash
ros2 launch demo_navigation navigation.launch.py
```

Arguments: `map`, `params_file`, `use_sim_time`, `autostart`, `use_composition`.

The launch delegates to `nav2_bringup`'s `bringup_launch.py` rather than
instantiating each server by hand: lifecycle-manager wiring and node ordering
are exactly what upstream maintains, and a hand-rolled copy would rot. What this
package owns is the parameter file, the map, and the namespace choices.

## docking_server

The params file configures `docking_server` even though the demo has no
charging dock. It is in Nav2 Jazzy's default node list, and without
`dock_plugins` it fails to configure — which makes the lifecycle manager
**abort the entire bringup**. The symptom is `map_server` and `amcl` reaching
`active` while planner, controller and bt_navigator sit in `inactive` forever.

Note that `docks` is *omitted*, not set to `[]`. An empty YAML list arrives at
launch as a Python tuple and aborts with
`Expected 'value' to be one of [float, int, str, bool, bytes], but got '()'`.

## Verifying

```bash
ros2 lifecycle get /bt_navigator          # expect: active
ros2 topic echo /demo/cmd_vel             # non-zero once a goal is sent
ros2 topic echo /local_costmap/costmap --once
```

Send a goal with RViz's **2D Goal Pose** tool, or:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
