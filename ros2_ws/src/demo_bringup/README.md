# demo_bringup

Top-level launch composition for the demo: one explicit launch file per
container role, plus the small set of commander and unit-adapter nodes that
sit between Nav2 and the plant.

**Runs on:** mixed, file by file — see the launch table below. `viz.launch.py`
and `cockpit.launch.py` are host-only; `sim.launch.py` is host-only in every
mode; `nav_select.launch.py` / `perception.launch.py` run on the x86 host in
`learn` mode and on the Aquila AM69 in `hil` mode; `learn.launch.py` composes
everything on the host. The commander nodes (`demo_routine`,
`patrol_commander`, `odom_tf`, `cmd_vel_si_to_stick`, `target_monitor`) are
machine-agnostic — they only speak ROS topics, never Gazebo or RViz.

## Overview

`demo_bringup` has no cross-mode conditionals: each launch file is the fixed
entrypoint of one Docker Compose service (`sim`, `nav`, `perception`, `viz`,
`cockpit`), and `learn.launch.py` is the native, non-containerized composition
used for development. The package also owns the nodes that translate between
subsystems — Nav2's SI velocities and the gait controller's stick units, raw
odometry and the TF tree, an exhibition choreography, and a supervised
patrol — because none of them belong to the plant (`demo_simulation`) or to
Nav2 itself.

## Nodes

| Node | Publishes | Subscribes | Services / actions |
| --- | --- | --- | --- |
| `wait_for_clock` | — | `/clock` | — |
| `wait_for_tf` | — | TF lookup (`can_transform`) | — |
| `demo_routine` | `/demo/cmd_vel` | `/demo/odom` | — |
| `odom_tf` | TF `odom → base` (+ optional static `map → odom` identity) | `/demo/odom` | — |
| `patrol_commander` | — | — | action client `navigate_to_pose` |
| `cmd_vel_si_to_stick` | `/demo/cmd_vel` | `/demo/cmd_vel_si` | — |
| `target_monitor` | `/demo/target/ops_log`, `/demo/target/status` | `/demo/cmd_vel_si`, `/demo/cmd_vel`, `/demo/odom` | — |

| Parameter | Default | Node | Description |
| --- | --- | --- | --- |
| `timeout_s` | `120.0` | `wait_for_clock` | Wall-clock deadline for `/clock` to start advancing. Non-zero exit on timeout. |
| `timeout_s` | `120.0` | `wait_for_tf` | Wall-clock deadline for the TF edge to appear. |
| `parent_frame` / `child_frame` | `odom` / `base` | `wait_for_tf` | Edge to probe with `can_transform`. |
| `rate_hz` | `20.0` | `demo_routine` | Publish rate; below ~7 Hz the gait controller's 0.3 s command watchdog makes the robot stutter. |
| `settle_s` | `5.0` | `demo_routine` | Silence between segments — silence is the stop command. |
| `cruise_mps` / `strafe_mps` / `turn_rps` | `0.10` / `0.08` / `0.10` | `demo_routine` | Measured, not aspirational, operating points (see Notes). |
| `min_z` / `stand_z` | `0.28` / `0.30` | `demo_routine` | Body-height gate: below `min_z` command publishing is suspended; above `stand_z` the routine starts. |
| `base_frame` / `odom_frame` / `map_frame` | `base` / `odom` / `map` | `odom_tf` | Frame names for the published edges. |
| `publish_map_identity` | `true` | `odom_tf` | Also publish a static `map → odom` identity. Set `false` once SLAM or AMCL owns that edge. |
| `waypoints` | triangle around `quadruped_objects.sdf` | `patrol_commander` | Flat `[x, y, yaw, …]` list in metres/radians; `yaw` is the arrival heading, not the departure heading. |
| `frame_id` | `map` | `patrol_commander` | Goal frame. |
| `goal_timeout_s` | `300.0` | `patrol_commander` | Per-goal deadline, derived from the Go2's measured average speed under MPPI avoidance. |
| `settle_s` | `2.0` | `patrol_commander` | Pause between goals so the gait controller can settle posture. |

`cmd_vel_si_to_stick` and `target_monitor` declare no parameters; their gains
(`0.4`/`0.3`/`0.5`) and topic names are module constants, matched to the gait
controller's `invNormalize` limits in `demo_simulation/config/gait_go2.yaml`.

## Launch files

| File | Starts | Key arguments |
| --- | --- | --- |
| `learn.launch.py` | Everything native on one host: sim → perception → navigation + RViz, timed | `world`, `map`, `rviz`, `navigation` |
| `sim.launch.py` | `sim` container entrypoint: includes `simulation.launch.py` or `quadruped.launch.py` per `robot_type`, plus a camera compressor | `world`, `gui`, `scene_cameras`, `robot_type` |
| `nav_select.launch.py` | `nav` container entrypoint: dispatches to `nav.launch.py` or `nav_quadruped.launch.py` | `robot_type` (default `quadruped`), `use_sim_time`, `params_override` |
| `nav.launch.py` | Diff-drive Nav2: `wait_for_clock` + `navigation.launch.py` (static map + AMCL) | `map`, `params_file`, `use_sim_time`, `clock_timeout_s` |
| `nav_quadruped.launch.py` | Go2 Nav2: `wait_for_clock`, `odom_tf`, `cmd_vel_si_to_stick`, `target_monitor`, `pointcloud_to_laserscan`, SLAM, `maze_explorer`, composed Nav2, `nav_control.launch.py`, `wait_for_tf` | `params_file` (default `nav2_params_go2.yaml`), `use_sim_time`, `clock_timeout_s`, `publish_map_identity` (default `false`) |
| `perception.launch.py` | `perception` container entrypoint: `wait_for_clock`, image decompressor, `demo_perception/perception.launch.py` | `use_sim_time`, `assumed_range_m`, `clock_timeout_s`, `detector_backend` |
| `viz.launch.py` | RViz2 alone, host only | `rviz_config`, `use_sim_time` |
| `cockpit.launch.py` | `cockpit` container entrypoint: `rosbridge_websocket` + `rosapi_node` + `web_video_server`. No `use_sim_time` on either node | `rosbridge_port` (9090), `video_port` (8080), `address` (`0.0.0.0`) |
| `routine.launch.py` | `demo_routine` alone, for exhibition | `rate_hz`, `settle_s`, `move_s`, `cruise_mps`, `strafe_mps`, `turn_rps`, `loop`, `min_z`, `stand_z` |
| `maze_nav_rviz.launch.py` | Host Nav2 + RViz click-to-goal viewer for the maze scenario (the plant is started separately) | `rviz_config` |

## Configuration

| File | What |
| --- | --- |
| `rviz/demo_view.rviz` | RViz config used by `learn.launch.py` and `viz.launch.py`; a `nav2_bringup` `nav2_default_view.rviz` derivative with `RobotModel` enabled and `TF` disabled by default (see Notes). |

## Build and test

```bash
cd ros2_ws
colcon build --symlink-install --packages-select demo_bringup
colcon test --packages-select demo_bringup && colcon test-result --verbose
```

## Notes

- **Startup ordering across a container boundary is a real failure mode, not
  a formality.** `wait_for_clock` and `wait_for_tf` replace the wall-clock
  timers `learn.launch.py` uses inside a single process tree; those timers
  mean nothing once `sim` and `nav` are separate containers started
  concurrently. Nav2 brought up before `/clock` is advancing, or before the
  `odom → base` edge exists, fails silently: lifecycle nodes stall or the
  lifecycle manager aborts bringup outright, and nothing in the logs names
  TF or the clock as the cause.
- **`/demo/cmd_vel` is not SI**, despite the `geometry_msgs/Twist` type — it
  carries normalized stick position. `cmd_vel_si_to_stick` is the boundary
  that lets Nav2 (which needs true SI to integrate MPPI rollouts) drive the
  same topic the exhibition and gait-trial tooling already use. Publishing
  Nav2 output directly to `/demo/cmd_vel` makes the robot move at ~40% of the
  commanded speed with no error anywhere.
- **`demo_routine` and `patrol_commander` must never run together.** Both can
  end up producing `/demo/cmd_vel`, and the robot jitters between whichever
  message arrived last with no diagnostic.
- **Do not point `rviz/demo_view.rviz` back at the upstream `nav2_bringup`
  config.** With `RobotModel` disabled and `TF` enabled, RViz shows no robot
  body and dozens of unlabelled axis triads instead — it looks like the robot
  fell apart, and it did not.
