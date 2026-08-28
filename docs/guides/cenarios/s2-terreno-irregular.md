# S2 — Rough terrain

World: `quadruped_rough.sdf` · x86 workstation · ~5 min

## Purpose

Measures foot placement (`FeetEndCalc`) and the state estimator when the support
points are not all on the same plane. This scenario puts the most stress on
**Defect 1**: the estimator integrates leg kinematics, and uneven support is
exactly where that integration makes the largest errors.

## Geometry and rationale

Seven 0,40 × 0,30 × **0,03 m** slabs between x = 1,2 and x = 5,4, alternating
±0,15 m off the centerline.

The 3 cm height is not arbitrary: gait foot clearance is 8 cm
(`gait.foot_height` in `gait_go2.yaml`), so the foot clears 3 cm comfortably
and the test measures the balance response to uneven support—not step-climbing
ability. A slab 8 cm or taller measures something else and will probably make
the robot fall.

They are offset so the left and right feet step at different heights at the
same time, which is the case of interest.

## Run

```bash
./scripts/run_quadruped_sim.sh quadruped_rough.sdf
./scripts/gait_trial.sh /tmp/s2.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 1 --walk 60 --hold 5
```

## Acceptance

Measured on 20/08/2026, RTF 1,00, 60 s of walking at 0,10 m/s. **Crosses.**

| measurement | S0 plane | S2 rough | acceptance |
| --- | --- | --- | --- |
| `RECOVER` | 0 | **0** | 0 |
| peak tilt while walking | 1,08° | **2,81°** | < 6° |
| maximum `footErr` | 0,021 m | **0,026 m** | < 0,04 m |
| average `footErr` | 0,003 m | 0,005 m | < 0,01 m |
| `z` | 0,343–0,359 m | 0,345–0,375 m | range < 4 cm |
| heading drift | −0,7° | −0,7° | < 5° |
| average speed | 0,1115 m/s | 0,1118 m/s | 97–115% |

The 3 cm slabs add ~1,7° of tilt and 5 mm of `footErr`, with no falls. The `z`
range grows by 1,6 cm as the body passes over the slabs.

### The result that matters: estimator degradation, quantified

| scenario | estimated Δ (x, y) | actual Δ (x, y) | y error | crabbing |
| --- | --- | --- | --- | --- |
| S0 plane | 3,972 · −0,007 | 4,330 · +0,138 | −0,145 m | 3,20% |
| **S2 rough** | 5,544 · +0,133 | 5,972 · +0,377 | **−0,244 m** | **6,31%** |

Crabbing **doubles** on rough terrain, and the estimator's y error doubles with
it. This numerically confirms that they are the same issue: Defect 1 is
leg-kinematics integration, and uneven support is where that integration makes
the largest errors. This is the reference scenario for any future estimator
work.

## What to inspect in the supervisor line

`footErr` is the metric for this scenario. On the empty plane it is symmetric
and below 2 cm; here it jumps when one foot encounters a slab and its diagonal
counterpart does not. Asymmetric and **persistent** `footErr` (not a spike)
indicates a stale support target, a known signature (§10 of the test guide).

## Scenario-specific pitfall

**Do not compare this scenario's lateral drift with S0 without inspecting
`estPos`.** Measured: crabbing doubles here (3,20% → 6,31%). This is **not**
gait degradation—the estimator is making twice the error, and the robot is
faithfully tracking that incorrect belief. Compare `estPos` in the supervisor
line with actual `/demo/odom` before changing any gain. A `k_y` adjustment "to
fix" this has already been measured on the plane and disproved
(`../../results/ml35-postura-parada.md`).
