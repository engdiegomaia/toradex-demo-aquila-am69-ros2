# S3 — Obstacle corridor

World: `quadruped_corridor.sdf` · x86 workstation · ~5 min

## Purpose

Exercises `/demo/scan` and the path F5 will use: scan → Nav2 costmap layer. It
**does not exercise gait:** the ground is intentionally flat so anything that
appears in gait figures can be attributed to S1/S2 rather than this scenario.

## Geometry and rationale

A 1,5 m-wide, 7 m-long corridor with walls 0,10 m thick and **0,60 m high**, an
end wall at x = 7,5, and two offset cylinders with 0,15 m radii at x = 2,5
(y = +0,25) and x = 5,0 (y = −0,25).

The 0,60 m height is the figure that matters. The Go2 lidar is on the body at
~0,33 m above the ground; a 0,30 m wall moves in and out of the scan plane as
the trunk oscillates, producing an **intermittent** scan that looks like a
bridge defect but is not. 0,60 m guarantees returns even while the body sways.

1,5 m is wide enough for Nav2 to plan and narrow enough for both walls to
appear in the same scan.

## Run

```bash
./scripts/run_quadruped_sim.sh quadruped_corridor.sdf
# check the lidar BEFORE walking
python3 scripts/scenario_check.py --seconds 20
# then walk through the corridor
./scripts/gait_trial.sh /tmp/s3.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 1 --walk 50 --hold 5
```

Direct scan inspection:

```bash
ros2 topic echo /demo/scan --once | head -20
ros2 topic hz /demo/scan
```

## Acceptance

Measured on 20/08/2026, RTF 1,00, 50 s of walking at 0,10 m/s.

| measurement | S0 plane | S3 corridor | acceptance |
| --- | --- | --- | --- |
| lidar: valid beams | 45/640 (**7%**) | **358/640 (56%)** | > 30% |
| lidar: minimum range | 4,66 m (ground) | **0,72 m** (wall) | < 1,5 m |
| lidar: maximum range | 9,76 m | 9,80 m | — |
| `RECOVER` | 0 | **0** | 0 |
| peak tilt while walking | 1,08° | 1,38° | < 3° |
| heading drift | −0,7° | −0,4° | < 5° |
| average speed | 0,1115 m/s | 0,0967 m/s | 97–115% |

The 7% → 56% change is this scenario's criterion. A minimum of 0,72 m confirms
that the side wall is seen at 0,75 m from the centerline, which is its location.

## Scenario-specific pitfalls

- **The robot will hit the end wall if it walks too long.** 50 s at 0,10 m/s is
  ~5 m; the wall is at 7,5 m. Reduce the duration if RTF is higher than expected
  or the command is greater.
- **~2% crabbing takes the robot toward the wall.** Over 5 m that is ~10 cm of
  lateral drift in a 1,5 m corridor—tolerable, but the reason this corridor is
  not narrower. Do not interpret the approach to the wall as a navigation
  failure: navigation is not running here.
- **An empty costmap is not a bridge defect if TF does not connect.** Without an
  `odom` frame, Nav2 cannot place the scan in a map. Check the tree before
  investigating the lidar.
