# S4 — Objects in the field of view

World: `quadruped_objects.sdf` · x86 workstation · ~4 min

## Purpose

Exercises `/demo/camera/image_raw` and `/demo/camera/camera_info`, as well as
the consumer of that contract, `demo_perception`.

## WARNING before reading any result

`demo_perception` is currently a **deterministic synthetic-detection stub**. It
**does not inspect the image**—it honors the topic contract in `CLAUDE.md` so
TIDL inference can be introduced later without refactoring.

Therefore, this scenario **does not validate detection**. It validates that the
image arrives with the correct geometry and rate, and that
`/demo/perception/detections` continues to be published and consumed. Confusing
the two is exactly how the stub would become "working vision" in a report.

## Geometry

Four objects in saturated colors distinct from the gray ground and blue
background:

| object | position (x, y) | shape |
| --- | --- | --- |
| red box | 1,5 · 0,0 | 0,30 m cube |
| green cylinder | 3,0 · +0,45 | r 0,18, h 0,50 |
| blue box | 3,0 · −0,55 | 0,40 × 0,40 × 0,60 |
| yellow cylinder | 4,5 · 0,0 | r 0,12, h 0,80 |

Three distances (1,5 / 3,0 / 4,5 m) give future inference targets at different
scales without requiring another world.

## Run

```bash
./scripts/run_quadruped_sim.sh quadruped_objects.sdf
python3 scripts/scenario_check.py --seconds 20
```

Walk only a short distance—the red box is 1,5 m ahead:

```bash
./scripts/gait_trial.sh /tmp/s4.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 1 --walk 8 --hold 5
```

To view the image with `demo_perception` running and the contract complete:

```bash
ros2 launch demo_perception perception.launch.py   # or the equivalent launch file
ros2 topic hz /demo/perception/detections
ros2 run rqt_image_view rqt_image_view /demo/camera/image_raw
```

`rqt_image_view` and RViz2 run **only on the x86 workstation** (rule 1 in
`CLAUDE.md`).

## Acceptance

Measured on 20/08/2026 with the robot stationary over a 15 s window.

| measurement | value | acceptance |
| --- | --- | --- |
| `/demo/camera/image_raw` | 10 Hz, 640×480 `rgb8`, 921600 bytes | > 5 Hz |
| geometry versus `camera_info` | matches | equal |
| average intensity | 179,4 (versus 180,9 in the empty world) | — |
| `RECOVER` | 0 | 0 |

The camera passes. **The lidar does not see the objects**—see below; this is the
scenario's main finding.

### The 2D lidar does not see an isolated obstacle ahead

| world | valid beams | minimum range |
| --- | --- | --- |
| S0 empty | 46/640 (7%) | 4,80 m (ground only) |
| **S4 objects** | **46/640 (7%)** | **4,82 m** |
| S3 corridor | 358/640 (56%) | 0,72 m |

S4 is **numerically identical to the empty world**: the four objects, 0,30 to
0,80 m tall and 1,5–4,5 m ahead, contribute **zero** returns.

Cause. The Go2 `L1_lidar` (`go2_description/xacro/gazebo.xacro:288`) is a **3D**
`gpu_lidar`: 640 horizontal samples over 200° across **16 vertical rings** at
±15°, mounted on `trunk` with `rpy="0 2.8782 0"`—164,9° of pitch, matching the
real L1 dome looking downward and forward.

The bridge maps it to `sensor_msgs/LaserScan`, which is **2D**. `/demo/scan`
carries 640 ranges, meaning **one of the 16 rings**; the other fifteen are
discarded during the crossing without warning. The exposed ring points such
that it sees a wall close to the body (S3, minimum 0,72 m) and ground at 5–10 m,
but not an isolated object ahead.

**Design consequence:** a Nav2 costmap fed by `/demo/scan` would fail to see the
very obstacles the robot must avoid. Solving this requires choosing either to
expose the lidar as `PointCloud2` (all 16 rings) instead of `LaserScan`, or to
remount/reorient the sensor. Neither has been done.

## Scenario-specific pitfall

**Camera at 0 Hz while its topic is listed** is the classic failure mode, and
the cause is almost always a world that does not load Gazebo's `Sensors`
system. Every world in this directory deliberately loads
`gz-sim-sensors-system` with `ogre2`; if you create a new world by copying
Gazebo's `empty.sdf`, the camera exists in the model but publishes nothing,
**with no error identifying the missing plugin**.
