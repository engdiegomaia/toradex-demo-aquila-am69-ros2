# `go2_description` — vendored package

Unitree Go2 quadruped description used by the simulation. **This package is
not ours.** It is third-party code copied into this tree, and this file
records where it came from, under what license, and exactly what was edited.

Same pattern as `demo_navigation/launch/nav2_vendored/`: change the minimum,
prove provenance, keep the diff visible.

---

## Why this package exists under this name

`robot.xacro` references meshes and includes through `$(find go2_description)`:

```xml
<mesh filename="file://$(find go2_description)/meshes/trunk.dae" scale="1 1 1"/>
<xacro:include filename="$(find go2_description)/xacro/const.xacro"/>
```

A ROS package named `go2_description` **has to exist** for this to resolve.
Placing the content as a subdirectory of `demo_description/urdf/` was tried
and does not work — `$(find)` cannot see a subdirectory. The alternative was
rewriting every `$(find go2_description)` to `$(find demo_description)`,
which would cost the "identical to upstream" property that the license
argument below depends on. Decision: keep the upstream name and do not edit
the references.

---

## Provenance — two layers, with different levels of proof

This is the part that matters. The two layers of this package **do not carry
the same weight of proof**, and the distinction is deliberate.

### Layer 1 — meshes: provenance proved

The 7 `.dae` meshes (25 of the package's 25 MB) come from:

- **Repository:** <https://github.com/unitreerobotics/unitree_ros>
- **Path:** `robots/go2_description/meshes/`
- **License:** BSD 3-Clause, full text in `LICENSE` in this directory
- **Copyright:** (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")

Proved by git blob hash (`git hash-object`), comparing the files in this tree
against the GitHub API `sha` values for `unitree_ros@master`:

| File here | git blob sha1 | File in `unitree_ros` |
|---|---|---|
| `meshes/calf.dae`         | `83221779392a5d19af59561d0ea4eea48ef6102a` | `calf.dae` |
| `meshes/calf_mirror.dae`  | `413fd75d8b173b4597a35cc71e618a7d025069bd` | `calf_mirror.dae` |
| `meshes/foot.dae`         | `22bfcf49d7b6d07015c88f55e4ca153463bd5d0e` | `foot.dae` |
| `meshes/hip.dae`          | `1bec1a72feb8a5d941ad6767341bc1d79d7eff0d` | `hip.dae` |
| `meshes/thigh.dae`        | `a17c0482f296f2e19849f2544bf309866adfb801` | `thigh.dae` |
| `meshes/thigh_mirror.dae` | `fb424112d623ad912f75248bedb1f62c6c28d3b6` | `thigh_mirror.dae` |
| `meshes/trunk.dae`        | `4e47b5bec4a3c91ae4a1d49d73080d697a917cf1` | **`base.dae`** |

**7 of 7 byte-identical.** The only difference: `base.dae` was renamed to
`trunk.dae` during `legubiao`'s repackaging; the content is bit-identical.

To reproduce:

```bash
# here
cd ros2_ws/src/go2_description/meshes && for f in *.dae; do
  printf "%s %s\n" "$(git hash-object "$f")" "$f"; done

# upstream
curl -sL "https://api.github.com/repos/unitreerobotics/unitree_ros/contents/robots/go2_description/meshes" \
  | grep -E '"(name|sha)"'
```

### Layer 2 — xacro and URDF: interpreted derivation, not proved

The files under `xacro/` and `urdf/robot.urdf` came from:

- **Repository:** <https://github.com/legubiao/quadruped_ros2_control>
- **Commit:** `5434c5810d1a7fe223bcfd04550e9d3bfdd4b458` ("x30 repaint", 2025-06-24)
- **Path:** `descriptions/unitree/go2_description/`
- **Declared license:** `<license>BSD</license>` in `package.xml` — **no
  license text, no copyright header, no identified author or maintainer**

These files **do not** match the `unitree_ros` upstream. They were rewritten
for ROS 2 / `ros2_control` (the upstream is ROS 1 / Gazebo Classic); file
sizes diverge across the board (`const.xacro` 5096 vs 7739 bytes, `leg.xacro`
7179 vs 12300, `robot.xacro` 4523 vs 4667).

**Reading adopted, and it is an interpretation:** these are a derivative work
of the Unitree description — same kinematics, same joint and link names, same
references to the same meshes — and are therefore covered by the upstream
BSD-3, with `legubiao`'s adaptation layered on top. The declared
`<license>BSD</license>` is consistent with that.

**Residual risk, recorded and not erased:** this coverage is not hash-proved
the way the meshes are. It is smaller than the risk that made the project
reject `a1_description` (which has no declared license and no traceable
holder), because here a proved holder exists for the primary asset and a
consistent BSD declaration exists for the derived layer — but it is not zero.

### What was rejected, and why

`descriptions/unitree/a1_description/` in the same repo declares
`<license>TODO</license>`. That is why the ML3.5 target moved from A1 to Go2.
See the project's engineering-log reference in `docs/engineering-log.md`.

---

## Edits made on top of upstream

Two, both deliberate.

### 1. Removal of what the demo does not use

Removed from the `legubiao` import (nothing the demo uses was touched):

| Removed | Why |
|---|---|
| `config/himloco/`, `config/legged_gym/`, `config/robot_lab/` | RL policy `.pt` weights, ~2.5 MB. The demo uses `unitree_guide_controller`, a classic PD controller that loads no policy at all |
| `config/ocs2/` | configs for `ocs2_quadruped_controller`, which is not part of the demo |
| `launch/` | upstream launch files (`gazebo_rl_control`, `visualize`). Ours live in `demo_simulation`, and the upstream launch brings up RViz2 inside it — rule 1 |
| `config/visualize_urdf.rviz` | upstream RViz2 config, same reason |
| original `README.md` | replaced by this file. Original content: build and launch instructions for the upstream repo, with relative links that do not resolve outside it |

From 28 MB to 25 MB. `meshes/`, `xacro/`, `urdf/` and the two configs the
simulation reads (`gazebo.yaml`, `robot_control.yaml`) are **untouched**.

### 2. `package.xml` — license and authorship

The only content file that was edited. Before:

```xml
<author>TODO</author>
<maintainer email="TODO@email.com"/>
<license>BSD</license>
```

After: author and copyright attributed to Unitree Robotics, license made
precise as `BSD-3-Clause` (the actual variant, confirmed in the upstream
`LICENSE`), maintainer pointing at this project — whoever maintains *this
copy* is us, and pretending otherwise would be worse. See the diff in git.

No file under `meshes/`, `xacro/`, `urdf/` or `config/` was edited. The
hashes in the table above are verifiable at any time and must keep matching.

---

## When updating this package

1. Redo the mesh hash comparison against `unitree_ros`.
2. If an xacro changes, record what and why here — do not edit silently.
3. Do not bring back `launch/` or the RL configs without a stated reason:
   `launch/` violates rule 1 (RViz2 inside a simulation launch file).
