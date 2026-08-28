# S1 — 6° ramp

World: `quadruped_ramp.sdf` · x86 workstation · ~5 min

## Purpose

Measures QP balance on an incline, the supervisor's `tilt` limit (`RECOVER`
above 12°), and body-height estimation when the support is not at z = 0.

## Geometry and rationale

A 4 × 2 × 0,2 m box inclined at 6°, with the **near top edge exactly at
z = 0, x = 1,0 m**. The ramp top is at x = 4,98 m, z = 0,42 m, followed by a
flat **plateau** from x = 4,98 through x = 7,0 at the height of the top.

The `<pose>` calculation in the SDF exists for a reason: a box simply rotated
about its own center leaves a step at the entrance, and the robot trips over it
instead of climbing the ramp—the test would then measure step traversal rather
than incline. The near end intentionally sinks below the ground plane so there
is no lip.

6° is deliberately modest: it is the first incline to measure, not the last.

**The plateau is not decoration.** The first version of this world ended the
ramp at a vertical face. The 20/08/2026 run showed the robot climbing 3,98 m
with tilt below 0,43° and then **falling off a 0,42 m cliff** at the upper edge:
peak tilt 155,95°, heading drift −162,7°, 84 `RECOVER`. The result looked like
"cannot climb the ramp" but was actually "has nowhere to go": failure occurred
at x = 4,968, or 3,97 m into a 3,98 m ramp.

This is a useful lesson about this harness: for a catastrophic result with
155° tilt, inspect *where* along the route failure began before drawing any
conclusion about the controller. The `x` versus `tilt` table answered that in
one line.

## Run

```bash
./scripts/run_quadruped_sim.sh quadruped_ramp.sdf
# in another terminal, with the ROS environment:
./scripts/gait_trial.sh /tmp/s1.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 1 --walk 60 --hold 5
```

60 s of walking: at 0,10 m/s the robot takes ~40 s to cover the 4 m ramp from
x = 1,0, and the margin accommodates RTF below 1.

## Acceptance

Measured on 20/08/2026, RTF 1,00, 70 s of walking at 0,10 m/s. **Climbs and
crosses the plateau.**

| measurement | S0 plane | S1 6° ramp | acceptance |
| --- | --- | --- | --- |
| `RECOVER` | 0 | **0** | 0 |
| peak tilt while walking | 1,08° | **0,96°** | < 4° |
| `z` | 0,343–0,359 m | **0,345 → 0,778 m** | rises 0,42 m with the terrain |
| heading drift | −0,7° | 2,8° | < 5° |
| average speed | 0,1115 m/s | 0,1063 m/s | 97–115% |
| net displacement | 4,33 m | 7,16 m | passes the top (x = 4,98) |

Peak tilt **below** the empty-plane figure is the result that matters: the 6°
incline costs nothing in balance. The ramp is not the limit; it is the baseline
from which to measure 10° and 15°.

Note the 7 m plateau. Both previous versions fell off edges—first the near one
(no plateau), then the far one (2 m plateau). If you increase walking time,
extend the plateau as well.

## Scenario-specific pitfalls

- **`z` increases, but the robot is not standing taller.** `/demo/odom` is an
  absolute pose in the world; while climbing the ramp, `z` rises with the
  terrain. Any script that uses `z` as a fall detector—including
  `demo_routine`—needs a terrain-relative threshold in this world, and the
  default `min_z` of 0,28 m becomes useless after the first meter of ascent.
- **The incline reduces crabbing, and that comes from the estimator.** Measured:
  2,38% on the ramp versus 3,20% on the plane and 6,31% on rough terrain. The
  gait is not improving uphill; leg-kinematics integration makes fewer errors
  when support is regular and inclined than when it is regular and level.
  Compare `estPos` with `/demo/odom` before drawing any gait conclusion.
- **Moving up to 10° and 15° is the next step, one variable per run.** Do not
  change the incline together with any gait gain; this project rule exists
  because four experiments have already been reverted for doing so.
