# demo_navigation

Nav2 configuration, maps, SLAM, frontier exploration, and the lifecycle-reset
façade for both robot paths of the demo.

**Runs on:** Aquila AM69 (arm64) in `hil` mode, x86 host in `learn` mode.
Architecture-neutral — this is the half of the demo that migrates to the
module. RViz is deliberately absent from every launch file here; it stays on
the host and is started separately by `demo_bringup`.

## Overview

Two independent Nav2 stacks live behind one package: static-map AMCL
navigation for the diff-drive robot (`navigation.launch.py`, delegating to
vendored `nav2_bringup` launch files), and reactive rolling-map navigation
plus SLAM-backed frontier exploration for the Go2 quadruped
(`nav_control.launch.py` + `slam.launch.py` + `maze_explorer`, driven from
`demo_bringup/launch/nav_quadruped.launch.py`). Both costmaps carry a
`perception_layer` reading the `PointCloud2` that `demo_perception` derives
from detections, so perception steers the robot rather than only decorating
the HMI.

## Nodes

| Node | Publishes | Subscribes | Services / actions |
| --- | --- | --- | --- |
| `nav_control_relay` | — | — | offers Trigger `/demo/nav/reset`, `/demo/nav/cancel`; calls `ManageLifecycleNodes`, `CancelGoal`, `/demo/exploration/cancel`, `ClearEntireCostmap` (both costmaps) |
| `maze_explorer` | `/demo/exploration/status` (String, transient-local) | `/map` (OccupancyGrid), `/demo/perception/maze_exit/pose` | offers Trigger `/demo/exploration/start`, `/demo/exploration/cancel`; action clients `compute_path_to_pose`, `navigate_to_pose`, `spin` |

| Parameter | Default | Node | Description |
| --- | --- | --- | --- |
| `manager_service` | `/lifecycle_manager_navigation/manage_nodes` | `nav_control_relay` | Lifecycle manager targeted by reset. Only the navigation manager — never the localization one. |
| `cancel_service` | `/navigate_to_pose/_action/cancel_goal` | `nav_control_relay` | Cancels all active goals (empty `goal_info`). |
| `costmap_services` | both `clear_entirely_*_costmap` services | `nav_control_relay` | Costmaps cleared before the PAUSE/RESUME cycle. |
| `exploration_bt_xml` | `''` (server default) | `maze_explorer` | Behavior tree override for `navigate_to_pose` goals. |
| `total_timeout_s` | `600.0` | `maze_explorer` | Overall exploration budget. |
| `goal_timeout_s` | `45.0` | `maze_explorer` | Per-goal deadline; measured against real stall/success timing, not a round number. |
| `nav_goal_tolerance_m` | `0.25` | `maze_explorer` | Must mirror the Nav2 `general_goal_checker`'s `xy_goal_tolerance`; a contract test keeps the two equal. |
| `min_frontier_distance_m` | `0.35` | `maze_explorer` | Frontier candidates closer than this to the robot are rejected — below Nav2's goal tolerance, Nav2 reports success without the robot moving. |
| `frontier_wall_clearance_m` | `0.38` | `maze_explorer` | Required clearance from occupied cells for a frontier target, matched to the Go2 footprint half-length. |
| `homing_max_distance_m` | `4.0` | `maze_explorer` | Entry gate for the blind approach to the maze-exit marker. |
| `homing_persistence_s` | `90.0` | `maze_explorer` | How long to keep walking to a latched marker pose after losing sight of it. |
| `stall_window_s` | `15.0` | `maze_explorer` | Motion watchdog during `navigating`; cancels a goal with no real progress. |
| `recovery_spin_rad` | `1.047` (~60°) | `maze_explorer` | Sweep used when no frontier cluster exists at all. |
| `breadcrumb_min_spacing_m` | `0.75` | `maze_explorer` | Minimum spacing for the visited-pose trail used to suppress revisits. |

`maze_explorer` declares roughly twenty parameters in total; the table above
covers the ones most likely to need tuning. See the module docstring in
`demo_navigation/maze_explorer.py` for the full set and the measurements
behind each default.

## Launch files

| File | Starts | Key arguments |
| --- | --- | --- |
| `launch/navigation.launch.py` | Vendored `bringup_launch.py` + `nav_control.launch.py` | `map`, `params_file`, `use_sim_time`, `autostart`, `use_composition` |
| `launch/nav_control.launch.py` | `nav_control_relay` | `nav_control` (enable/disable) |
| `launch/slam.launch.py` | `slam_toolbox` async, as a lifecycle node driven through `configure` → `activate` | `use_sim_time`, `params_file`, `scan_topic` |
| `launch/nav2_vendored/{bringup,localization,navigation,slam}_launch.py` | Verbatim `nav2_bringup` copies — see `launch/nav2_vendored/README.md` | (unchanged from upstream) |

## Configuration

| File | What |
| --- | --- |
| `config/nav2_params.yaml` | Nav2 for the diff-drive robot (`robot_base_frame: base_footprint`, `robot_radius: 0.25`, velocity limits matched to a 0.35 m wheel separation). |
| `config/nav2_params_go2.yaml` | Default Go2 Nav2 params: frame `base`, polygon footprint ±0.37×±0.18 m, `/demo/scan_cloud` source, MPPI controller, `consider_footprint: true`. |
| `config/nav2_params_go2_footprint.yaml` | Historical alias, value-equivalent to `nav2_params_go2.yaml` (parsed YAML is identical; comments differ). Kept because `NAV2_PARAMS` overrides and older reports reference it. |
| `config/params-align8.yaml` | A/B evaluation baseline arm used by `tools/evaluation/nav_campaign.py`; still live, header text is stale (see file). |
| `config/slam_params.yaml` | `slam_toolbox` parameters for the live Go2 map. |
| `maps/warehouse.{pgm,yaml}` | Static map for the diff-drive path (477 × 475 cells at 5 cm). Committed; regenerate only if the world changes. |
| `behavior_trees/nav_to_pose_exploration.xml` | Behavior tree used during frontier exploration. |
| `behavior_trees/nav_to_pose_smoothed.xml` | Nav-to-pose behavior tree with path smoothing. |

## Build and test

```bash
cd ros2_ws
colcon build --symlink-install --packages-select demo_navigation
colcon test --packages-select demo_navigation && colcon test-result --verbose
```

## Notes

- **`docking_server` must stay configured even with no charging dock.** It is
  in Nav2 Jazzy's default managed-node list; without `dock_plugins` it fails
  to configure, which makes the lifecycle manager abort the entire bringup —
  the symptom is `map_server`/`amcl` reaching `active` while the planner,
  controller and `bt_navigator` sit `inactive` forever. `docks` must be
  *omitted*, not set to `[]` — an empty YAML list arrives as a Python tuple
  and aborts with a type error.
- **`slam_toolbox` is a lifecycle node that starts `unconfigured` and looks
  alive while doing nothing** (no scan subscription, no map, no error).
  `slam.launch.py` drives the `configure`/`activate` transitions explicitly;
  verify with `ros2 lifecycle get /slam_toolbox` (must read `active`).
- **The SLAM scan topic is set by remap, not by parameter.** `slam_toolbox`
  reads `scan_topic` while declaring parameters, so an external params file
  does not reliably take effect. `slam.launch.py` remaps `/scan → /demo/scan`
  instead; verify with `ros2 node info /slam_toolbox`.
- **`nav_control_relay` resets by PAUSE + RESUME, never RESET + STARTUP.**
  Measured: a `ManageLifecycleNodes` RESET followed by STARTUP crashes the
  composed Nav2 container with SIGSEGV in `nav2_route`'s `route_server`, a
  node this project loads (because it is in the vendored launch file's
  managed-node list) but never calls. PAUSE/RESUME skips the `CONFIGURE`
  transition that triggers the crash, at the cost of not re-reading the
  behavior tree — which never changes during a demo anyway. Costmaps are
  cleared explicitly as a separate step, since PAUSE/RESUME does not reset
  accumulated obstacle state.
- **Only the navigation lifecycle manager is reset, never the localization
  one.** Pausing AMCL mid-demo turns "navigation stalled" into "the robot no
  longer knows where it is." The Go2 path has no localization manager at all —
  `odom_tf` publishes `map → odom` as a static identity.
- **The perception costmap layer** (`perception_layer`, both local and
  global costmaps) reads `/demo/perception/detection_cloud` with `clearing:
  false` and `observation_persistence: 1.0`, because the projection behind it
  has no depth — see `demo_perception/README.md`. This wiring is deliberately
  left unchanged when the perception stub is replaced by real inference.
