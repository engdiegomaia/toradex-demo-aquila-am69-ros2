# Provenance — vendored control stack

Covers the four control packages vendored together:

- `control_input_msgs`
- `controller_common`
- `unitree_guide_controller`
- `gz_quadruped_hardware`

**None of these are ours.** Same pattern as `go2_description/README.md` and
`demo_navigation/launch/nav2_vendored/`: change the minimum, prove
provenance, keep the diff visible.

---

## Origin

- **Repository:** <https://github.com/legubiao/quadruped_ros2_control>
- **Commit:** `5434c5810d1a7fe223bcfd04550e9d3bfdd4b458` ("x30 repaint", 2025-06-24)
- **Paths:** `commands/control_input_msgs`, `libraries/controller_common`,
  `controllers/unitree_guide_controller`, `hardwares/gz_quadruped_hardware`

Extracted from a shallow clone of that commit. Upstream names were preserved:
the `$(find <package>)` references in the xacro and launch files resolve
without edits, and the diff against upstream is nil.

---

## License — what the audit found

The `package.xml` files declare `Apache-2.0` (or `Apache 2`), but actual
coverage is mixed. Measured, not assumed:

| Package | `package.xml` declares | Actual coverage | Holder |
|---|---|---|---|
| `gz_quadruped_hardware` | `Apache 2` | own `LICENSE`, Apache headers in **5/5** sources | Open Source Robotics Foundation |
| `unitree_guide_controller` | `Apache-2.0` | `LICENSES/unitree_guide/LICENSE.txt` at the upstream repo root — **BSD 3-Clause** | Unitree Robotics |
| `controller_common` | `Apache-2.0` | same (code extracted from `unitree_guide_controller`) | Unitree Robotics |
| `control_input_msgs` | `Apache-2.0` | same; message definitions only, 20 KB, no C++ source | Unitree Robotics |

### What this means

`gz_quadruped_hardware` is a **fork of OSRF's `gz_ros2_control`**, and it is
the clean case: its own `LICENSE` in the package, and a full Apache header in
all five sources. Nothing to resolve.

The other three derive from Unitree's **`unitree_guide`**, ported to ROS 2 by
`legubiao`. Most of the sources carry no copyright header — only
`// Created by tlab-uav on 24-9-6.` — which reads as a missing license on a
naive audit. It is not: the upstream repository keeps
`LICENSES/unitree_guide/LICENSE.txt` at its root **specifically** to cover
this code, and the text is BSD 3-Clause, copyright (c) 2016-2022 HangZhou
YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics").

Independent confirmation:
`include/unitree_guide_controller/common/mathTypes.h` is the only source that
carries a header, and it reads
`Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.` —
the same holder.

This is **the same holder and the same license** as the `go2_description`
meshes, whose identity with `unitreerobotics/unitree_ros` is hash-proved. The
demo's two layers (description and control) converge on the same origin and
the same BSD-3 license.

### Contrast with the `a1_description` case

Here, license text and an identified holder both exist — which is what
`a1_description` lacked (`<license>TODO</license>`, no text, no copyright),
and which is why the ML3.5 target moved from A1 to Go2. BSD-3 clause 1
(retain the copyright notice on source redistribution) is satisfiable: the
`LICENSE` is present in each package.

---

## Edits made on top of upstream

### 1. `LICENSE` added to three packages

`control_input_msgs`, `controller_common` and `unitree_guide_controller`
received a full copy of `LICENSES/unitree_guide/LICENSE.txt` from the
upstream repository. Upstream keeps that text only at the root; when the
packages are extracted individually it has to travel with them, or the
redistribution fails clause 1.

`gz_quadruped_hardware` already carried its own and was not touched.

### 2. `package.xml` — license made precise

In the three packages derived from `unitree_guide`,
`<license>Apache-2.0</license>` was corrected to
`<license>BSD-3-Clause</license>`, which is what the text actually covering
them says. Declaring Apache over BSD-3 code would be wrong in both
directions.

`gz_quadruped_hardware` had `Apache 2` normalized to `Apache-2.0` (SPDX
spelling). Holder and content unchanged.

### 3. `unitree_guide_controller` source edited from F4 onward

This **changed** after vendoring, and an earlier version of this file said
the opposite. "No source file was edited" was true as of the initial
vendoring and stopped being true the next day, when tuning work on the gait
began. Recorded here instead of silently corrected.

Edited in `unitree_guide_controller`, each with the measured number and
reason documented as a comment at the point of use:

| File | What changed |
|---|---|
| `src/FSM/StateTrotting.cpp` + `.h` | rewritten: WALK/HOLD/RECOVER modes, attitude supervisor, reference band sized to the command, yaw-axis diagnostics and instrumentation |
| `src/control/BalanceCtrl.cpp` + `.h` | Go2 inertia in place of A1's; QP weights and friction cone taken from parameters |
| `src/gait/FeetEndCalc.cpp` | heading gain `k_yaw` 0.005 → 0.15; the three Raibert gains taken from parameters |
| `src/gait/GaitGenerator.cpp` + `.h` | support target re-anchored on touchdown and while the gait is stopped |
| `src/control/Estimator.cpp` + `.h` | state access used by diagnostics |
| `src/UnitreeGuideController.cpp` + `.h` | declaration and validation of the gait parameters |
| `include/.../control/GaitParams.h` | **new file, ours**: the tuning surface |

Full history: `git log ae3d9a1..HEAD --
ros2_ws/src/unitree_guide_controller/`; the evidence behind each change is
preserved in the archived engineering log (see `docs/engineering-log.md`).

**What remains untouched, and why it matters:** `src/quadProgpp/`
(third-party solver), `CMakeLists.txt`, `package.xml` beyond the license, and
the plugin XML. And, outside this package, all of `go2_description/` —
nothing under `meshes/`, `xacro/`, `urdf/` or `config/` was touched. That is
why the gait tuning went into `demo_simulation/config/gait_go2.yaml`,
injected by the spawner, instead of into
`go2_description/config/gazebo.yaml`: that package's license argument
depends on it staying byte-identical to upstream.

No CMake or xacro file was edited in any of the four packages.

---

## Why these four, and not the whole repository

The upstream repository ships 24 packages. Only the ones the demo uses are
included:

- `unitree_guide_controller` — classic PD gait controller, no RL policy. This
  is what made the F2 gate pass.
- `controller_common` — library the controller depends on.
- `control_input_msgs` — the `Inputs` type the controller consumes.
- `gz_quadruped_hardware` — the `ros2_control` plugin that runs inside the
  `gz sim` process. It is the plugin *from this repository*, version 2.0.6,
  not the apt `gz_ros2_control` 1.2.19. The original plan assumed the apt
  package; installing the apt package and expecting the base image to use it
  was an unverified assumption.

Deliberately left out: `ocs2_quadruped_controller` and
`rl_quadruped_controller` (controllers the demo does not use),
`hardware_unitree_sdk2` (the physical-robot SDK — and where the CycloneDDS ×
`unitree_sdk2` collision lives; see `docker/hw/README.md` and
`docs/engineering-log.md`), `unitree_joystick_input`, and every other robot's
description.

---

## When updating these packages

1. Re-check whether the upstream root `LICENSES/` still covers what is
   brought in.
2. If a source gains a copyright header upstream, it supersedes this table —
   update it here.
3. Do not bring in `hardware_unitree_sdk2` without first resolving the RMW
   collision: project rule 2 is `rmw_cyclonedds_cpp` always, and the SDK
   expects Fast DDS.
