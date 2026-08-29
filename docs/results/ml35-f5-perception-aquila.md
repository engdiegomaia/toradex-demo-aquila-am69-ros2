# ML3.5 F5 — perception gate on the Aquila AM69

Date: 28/08/2026. Commit under test: `dd98a89`. Images: `local/demo-aquila-{nav,perception}:dev`,
built on the module, unchanged since the last functional commit (`8676941` touched only
`package.xml`/`setup.py` description strings).

Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + perception on the
Aquila AM69 over `enp0s31f6`, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`,
`lifecycle_manager_navigation/is_active` = `success=True`.

Raw samples: `ml35-f5-perception-aquila.csv`.

> **Verdict: CLOSED — PASS.** The transport chain passed on 28/08. The
> positive-detection and pose criteria were reached on **29/08/2026** once the robot
> was placed in line of sight of the marker: **60 of 60 detection messages non-empty,
> 5/5 on the first five frames**, pose published and finite, `front_camera` resolvable
> to `map`, `odom` and `base` at the pose timestamp, TF 99.83%. Section 6 records that
> run. The 28/08 shortfall was never a perception defect — it was line of sight, and
> the sections below are kept as the record of how that was established.

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

*(Written 28/08, before the closure. Kept as stated.)* The transport half needs no
further work. The remaining half needs the robot to reach the exit opening, which is
blocked by the defect recorded in `ml35-f5-exploration-smoke.md` §3. No perception
change is indicated by this run.

---

## 6. Closure — 29/08/2026: the detector fires, measured

The exploration defect is still open, so waiting for the explorer to deliver line of
sight would have kept two gates blocked on one bug. Instead the robot was **placed**
at the exit region and perception measured on its own. Manual placement is prohibited
inside the exploration smoke; it is not prohibited in a perception gate, and
decoupling the two is what let this one close.

### How the robot got there

Not a raw teleport. `SetEntityPose` preserves velocity, which drops a trotting robot
0.15 m and slides it — the failure already documented in `sim_control_relay.py`. The
safe sequence that node defines was used instead:

```
/demo/gait/hold            -> "gait: trotting -> fixed stand, robô imóvel"
sleep 3 s
/demo/sim/set_entity_pose  -> demo_robot to (-4.90, -1.30, 0.40), yaw -90°
sleep 2 s
/demo/gait/resume          -> "gait: fixed stand -> trotting, pose reancorada"
```

Resulting pose, from `/demo/odom`: `(-4.902, -1.300, 0.362)`, quaternion
`z=-0.70713, w=0.70708` — on the opening axis `OPENING_X = -4.90`, inside the maze
boundary `BOUNDARY_Y = -0.90`, facing the marker 1.30 m away. The robot stood; it did
not fall.

### The criteria, scored

| §1 criterion | measured | verdict |
| --- | --- | --- |
| detection in ≥3 of 5 frames | **5/5**; 60/60 over the whole capture | **pass** |
| `class_id` / score | `maze_exit`, score 1.0 | **pass** |
| bounding box above `min_bbox_width_px=12` | 149 × 153 px, centre (318, 200) | **pass** |
| pose published | 59 messages alongside 60 detections | **pass** |
| pose `frame_id` | `front_camera` | **pass** |
| pose values finite | `(0.992, 0.011, 0.000)` | **pass** |
| TF at the pose timestamp | `map`, `odom` and `base` all resolved | **pass** |
| detection rate | 9.733 / 9.779 Hz | **pass** |
| TF ≥ 99.5% under detector load | **99.83%** over 60 s, RTF 0.957 | **pass** |

Transforms resolved at the pose stamp:

```
map  <- front_camera   (-5.034, -1.346,  0.003)
odom <- front_camera   (-4.900, -1.627,  0.402)
base <- front_camera   ( 0.327,  0.000,  0.043)
```

`base <- front_camera` = 0.327 m forward is the camera offset, and it is what makes
the pose figure checkable.

### Range is biased ~21% short, and the cause is known

The camera sits at `map` y = -1.346; the marker is at y = -2.60. True camera-to-marker
distance is **1.254 m**. The detector reported **0.992 m** — 0.262 m short, 21%.

The estimator is `range = fx · marker_width_m / width_px`, with `fx = 184.836`
(from `camera_info`) and `marker_width_m = 0.8`. Geometry predicts a 117.9 px wide
blob at 1.254 m; the detector measured 149 px, 26% wider. So the bias is in the blob,
not the arithmetic: the panel's material carries `emissive 0.35 0.0 0.35`, and the
glow crosses the magenta threshold beyond the panel's geometric edge.

Consequence for homing, stated rather than fixed: `marker_stop_distance_m = 0.7` is
compared against this estimate, so the robot will stop at roughly **0.55 m true**
instead of 0.70 m. The marker is a thin static visual with no collision geometry, so
this is a logged limitation, not a blocker. **No detector parameter was changed** —
the protocol authorises changing only `sample_stride`, and only if CPU interferes.

### CPU did not interfere, again

With the marker in view and the detector on its positive path, TF held at **99.83%**,
above the 99.5% floor. Module load average 17.4 across 8 cores; `demo-perception-1` at
279% and `demo-nav-1` at 407% of a single core. High, and worth watching during the
exploration rounds — but by the gate's own criterion, not interfering. `sample_stride`
stays at 4.

### What section 6 still does not validate

- Still Gazebo images on the x86 host, so still nothing about a physical Go2.
- The robot was placed, not driven. This proves the detector and the TF chain; it
  proves nothing about the explorer reaching the exit on its own.
- `homing_exit` was not exercised: the explorer was `idle` throughout, deliberately,
  so that this run changed exactly one thing.
