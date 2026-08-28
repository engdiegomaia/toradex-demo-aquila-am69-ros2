# ML3.5 F5 — perception gate on the Aquila AM69

Date: 28/08/2026. Commit under test: `dd98a89`. Images: `local/demo-aquila-{nav,perception}:dev`,
built on the module, unchanged since the last functional commit (`8676941` touched only
`package.xml`/`setup.py` description strings).

Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + perception on the
Aquila AM69 over `enp0s31f6`, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`,
`lifecycle_manager_navigation/is_active` = `success=True`.

Raw samples: `ml35-f5-perception-aquila.csv`.

> **Verdict: the transport chain PASSES on the module. The positive-detection and
> pose criteria were NOT REACHED, because the robot never obtained line of sight to
> the marker.** This is not a perception failure — it is a mobility and exploration
> failure, measured separately in `ml35-f5-exploration-smoke.md`. The gate stays
> open.

---

## 1. What was proven

The image path crosses the Ethernet link to the module and the detector consumes it.

| topic | measured (window 50) | jitter (std dev) |
| --- | --- | --- |
| `/demo/camera/camera_info` | 9.697 / 9.666 Hz | 6.1 ms |
| `/demo/perception/image_in` | 6.996 / 7.028 Hz | 111 ms |
| `/demo/perception/maze_exit/detections` | 9.352 / 9.836 Hz | 77 ms |

`camera_info` reaches the module, so `marker_pose()` has intrinsics available — that
was the first listed blocker and it is cleared.

The detector subscribes to `/demo/camera/image_raw`, which
`perception.launch.py` remaps to `/demo/perception/image_in`; the module runs a local
`republish` from `/demo/camera/image_raw/compressed`. That indirection is why the
gate measures `image_in` and not `image_raw`: on the module, `image_raw` is not the
topic the detector reads.

`detections` is published at ~9.6 Hz — one message per processed image, including
empty ones. Detector parameters in force: `sample_stride=4`, `min_samples=40`,
`min_bbox_width_px=12`, `confirm_frames=3`, `marker_width_m=0.8`.

### The `image_in` rate reads lower than `detections`, and that ordering is an artefact

7.0 Hz of images cannot produce 9.6 Hz of detections. The measurement, not the
pipeline, is responsible: `ros2 topic hz` on `image_in` adds a second subscriber to an
**uncompressed** image topic inside the module, and the 0.647 s maximum interval in
that sample is the probe's own cost. The `detections` rate matches `camera_info`
(9.7 Hz), which is the honest figure for what the detector actually processes. Do not
quote 7.0 Hz as the detector input rate.

## 2. What was NOT proven, and why

The protocol asks for at least three positive detections in five messages, one
published pose, its `frame_id`, a camera-to-`map` transform at the pose timestamp,
finite values and a plausible range. **None of these were reachable**, for a
geometric reason rather than a software one.

- The exit marker is a static magenta panel at world `(-4.90, -2.60, 0.60)`,
  0.80 × 0.03 × 0.80 m, its wide faces normal to world *y*.
- `maze_escape_validator` places the exit opening at `x = -4.90`, `y = -0.90`,
  half-width 0.60 m. The panel therefore sits **1.70 m outside the maze**, in line
  with the opening. From inside the maze it is visible only from close to the
  opening, looking in −*y*.
- With the stack up for four hours, the robot was at `(-1.72, 3.00)`, 6.4 m from the
  panel with maze walls in between. A bounded in-place sweep — 50 s at 0.15 rad/s,
  more than a full turn, commanded on `/demo/cmd_vel_si` with Nav2 idle — produced
  **445 detection messages, 445 of them empty**.
- The exit region was entirely unknown in `/map` (every probed cell at `(-4.9, ±)`
  returned `-1`), so a manual Nav2 goal to the opening had nothing to plan through.

Empty detections in front of no panel are the correct output. The criterion needs the
robot to reach the opening, and that is what the exploration run failed to do.

Only the frame could be checked: the detection header carries `frame_id:
front_camera`, which is the project's camera frame.

## 3. CPU impact — measured, and not a blocker by the gate's own criterion

Captured on the module at 20:23:48, with exploration active (`top -b -n 2 -d 3`,
PIDs resolved against `/proc/<pid>/cmdline`):

| process | %CPU (of 800%) |
| --- | --- |
| `component_container_isolated` (Nav2) | 229.5 |
| `maze_exit_detector` | 89.1 |
| `detection_stub` | 79.8 |
| `maze_explorer` | 76.8 |
| `detections_to_cloud` | 71.5 |
| `camera_decompressor` (`republish`) | 31.8 |
| `pointcloud_to_laserscan` | 23.5 |
| `target_monitor` | 19.2 |
| `odom_tf` | 18.5 |
| `async_slam_toolbox` | 17.2 |

Total ≈ 657% of 800%; 1-minute load average 19.68 on 8 cores.

`maze_exit_detector` at 89.1% is the largest single Python consumer, which is what a
per-pixel loop in Python at stride 4 costs. **It did not breach any gate criterion:**

| criterion | measured | verdict |
| --- | --- | --- |
| TF availability ≥ 99.5% | 99.93% (2867 samples, 300 s, concurrent with the run) | pass |
| detector still producing detections | ~9.6 Hz throughout | pass |
| no sustained controller loss | real-time factor 0.956; no deadline errors in log | pass |
| no `invalid source` regression | 0 occurrences in a 15-minute log window | pass |

The protocol says to change `sample_stride` only if detector CPU interferes. By the
protocol's own measurements it did not, so **`sample_stride` was left at 4**. The load
average is high and worth revisiting, but no threshold in this gate justifies the
change, and changing it would have made the exploration run non-comparable.

TF was also measured before the run: 99.74% over 389 samples / 40 s, median age
30 ms, `base <- lidar` 100%, no future stamps.

## 4. What this does not validate

- Nothing here validates a physical Go2: the images come from Gazebo on the x86 host
  (`CLAUDE.md` rule 7).
- The CPU figures are real AM69 measurements under a real workload, but they describe
  this distributed HIL stack, not the module in isolation.
- The detector's positive path remains **unexercised on the module**. It is covered by
  `test_maze_exit_confirmation.py` at unit level only.

## 5. To close this gate

The transport half needs no further work. The remaining half needs the robot to reach
the exit opening, which is blocked by the defect recorded in
`ml35-f5-exploration-smoke.md` §3. No perception change is indicated by this run.
