# Validation Status

This page summarizes what has been validated, **where** it was validated, and
what is still pending. It condenses the project's engineering log, which is
preserved in full at the git tag `archive/ml35-engineering-log` (see
[engineering-log.md](engineering-log.md)).

## Validation policy

| Environment | What it can prove | What it cannot prove |
| --- | --- | --- |
| **Toradex Aquila AM69** (real module, Torizon OS 7.7.0) | CPU load, latency, message rates, DDS behaviour across the physical link, thermals | Anything about a physical legged robot: the plant is always simulated |
| **x86 host** (Gazebo Harmonic) | Functional behaviour, gait tuning, navigation logic, cockpit UI | Performance on the target |
| **QEMU arm64 emulation** | That the arm64 images build | Nothing about performance. No CPU, latency, thermal or FPS figure is ever taken from QEMU |

In every hardware-in-the-loop (HIL) result below, Nav2 and perception run on the
real Aquila AM69. The robot is a Unitree Go2 **simulated** in Gazebo on the x86
host. No physical robot has been part of any test.

## Current release status

**Supervised navigation-and-exploration demo.** The Go2 model walks, maps the
maze with SLAM, explores it autonomously with Nav2 and a frontier explorer, and
the whole loop is operated from the web cockpit. Navigation and perception run
on the Aquila AM69 during all of this.

The release does **not** yet demonstrate a complete autonomous maze escape. The
escape validator (`/demo/maze/escaped`) has not reported `true` in any recorded
round. The planned acceptance run has not been executed: three cold starts,
each escaping within 600 s.

## Results on the Aquila AM69 (real hardware)

| Date (2026) | Result |
| --- | --- |
| 20 Aug | Module inventoried: 8× Cortex-A72, 31 GiB RAM, Docker 25.0.9, Compose 2.26.0. arm64 images built natively on the module (`nav` 2.44 GB, `perception` 1.28 GB, `tools` 1.32 GB, `base` 1.24 GB). |
| 21 Aug | First HIL over Wi-Fi: Nav2 active on the module, bidirectional DDS with the host simulator. Bottleneck: the raw camera stream (74.2 Mbit/s, 640×480 @ 10 Hz). |
| 24 Aug | HIL over wired Gigabit Ethernet (RTT 0.40 ms). Short goal succeeded in 28 s. Both 8 m goals timed out (Nav2 493–530 % CPU, perception 189–205 %, of 800 %). |
| 25 Aug | Compressed camera transport: wire traffic reduced 82× (9.3 MB/s → 113.5 KB/s), module CPU 711 % → 600 %. |
| 27 Aug | Root cause of the long-goal timeouts found: goal geometry that crossed unmapped walls made the global plan oscillate. With connected routes, **8/8 goals succeeded** on unchanged hardware and parameters. |
| 28 Aug | Perception on the module: camera info at 9.70 Hz, detections at ~9.6 Hz, detector 89 % of one core, TF availability 99.93 %. |
| 28 Aug | Joint-state broadcaster at 50 Hz instead of 1000 Hz: `/tf` 1054 → 145 Hz, `odom→lidar` TF availability 94.75 % → **99.94 %**, module CPU −31 points. |
| 29 Aug | Exit-marker gate: 60/60 detections at a placed vantage point, TF 99.83–99.94 %. Known bias: range reads 21 % short (bloom from the emissive material). |
| 29 Aug | Footprint A/B: a rectangular trunk footprint replaced `robot_radius`. The robot's own costmap cell stayed ≤ 165 (was up to 243, fatal is 253), and exploration survived the full 600 s instead of stopping at 116 s. |
| 26 Aug | Target telemetry through the cockpit: CPU 54.5 %, SoC 35.0 °C, memory 4.4 %. Measured once, with the module lightly loaded; not a thermal characterization. |
| 28–30 Aug | Exploration rounds R1–R16 in HIL. Best: R5 travelled 41.4 m. R13 completed 16/23 goals with zero falls in 642 s. R12 got closest to the exit, at (−5.08, 1.47). R16 had the smoothest motion of any round (median commanded yaw rate 0.031 rad/s) with 60 % goal completion. |

## Results on the x86 host (simulation only)

- **Gait:** the forward envelope is 0.20 m/s linear and 0.13 rad/s angular. Two defects were fixed:
  - Curved walking, fixed with `k_yaw` 0.15 → 0.25. Lateral drift fell from 1.1–1.6 m to 0.08–0.11 m.
  - Falls while standing still, fixed with `hold.settle_rate = 0.02`. The robot now stands for more than 180 s.
- **Navigation tuning on the maze:** mean speed +63 % (0.040 → 0.065 m/s), reverse driving 11–62 % → 0 %, path efficiency 13 % → 57 %.
- **Maze geometry:** checked offline. The corridors are 1.20 m wide, the navigable area is 35.4 m² and forms a single connected component.
- **Web cockpit:** validated in `learn` mode with Firefox (headless). This covers the five live panels, click-to-goal, simulation play/pause/reset and scene-camera control.

## Test suites

The unit and contract tests are listed in [development.md](development.md#tests).
They run on the host and need no hardware.

## Pending

| Item | Status |
| --- | --- |
| Autonomous escape: 3 cold starts, each with `escaped=true` in ≤ 600 s | **PENDING EXECUTION** |
| Sustained traversal ≥ 0.05 m/s over a real ≥ 8 m route | **PENDING EXECUTION** (only the short stability gate, 3 goals × 3 runs, has passed) |
| Fall risk while homing blindly towards the exit marker | Root cause is a hypothesis; **not mitigated** |
| Sweep recovery when no frontier remains | Implemented and unit-tested; never triggered in HIL |
| SLAM-derived static layer for the global planner | Not implemented |
| Chromium kiosk HMI on the Aquila AM69 display | Not started |
| Cockpit manual control behind `twist_mux` | Designed; the arrow buttons are intentionally inert |
| Sustained thermal and power characterization | **PENDING EXECUTION** |
| TIDL / NPU inference | Out of scope; perception is a deterministic stub |
| Physical Go2 hardware | Out of scope |

## Methodological caveat

Every exploration round explores the maze differently. A/B comparisons made
from single rounds (n = 1) therefore cannot separate a real tuning effect from
ordinary run-to-run variance. Any future controller tuning should use repeated
runs under matched conditions.
