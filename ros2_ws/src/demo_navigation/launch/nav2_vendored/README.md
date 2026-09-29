# Vendored Nav2 launch files

Verbatim copies of four launch files from `nav2_bringup`, with **only** the
package-root paths re-rooted. Copyright headers are intact.

**Runs on:** Aquila AM69 (arm64) in `hil` mode, x86 host in `learn` mode —
same as the rest of `demo_navigation`.

| File | Origin |
| --- | --- |
| `bringup_launch.py` | `nav2_bringup/launch/bringup_launch.py` |
| `localization_launch.py` | `nav2_bringup/launch/localization_launch.py` |
| `navigation_launch.py` | `nav2_bringup/launch/navigation_launch.py` |
| `slam_launch.py` | `nav2_bringup/launch/slam_launch.py` |

**Source:** `ros-jazzy-nav2-bringup`, ROS 2 Jazzy, Ubuntu 24.04 arm64/amd64
apt packages.
**License:** Apache-2.0 (Copyright 2018 Intel Corporation and Nav2
contributors), the same license this project uses.

## Why these are vendored

`demo_navigation/launch/navigation.launch.py` delegates to `bringup_launch.py`
on purpose: the lifecycle-manager wiring and node ordering are what upstream
maintains, and hand-duplicating them here would drift. What could not stand is
installing the `ros-jazzy-nav2-bringup` **package** into the `nav` container
image. `nav` is one of the two images that travels to the Aquila AM69, and
`nav2-bringup` hard-depends on `nav2-minimal-tb3-sim`, `nav2-minimal-tb4-sim`,
`ros-gz-sim`, `ros-gz-bridge`, and `navigation2` (which pulls
`nav2-rviz-plugins`). Those pull in the entire Gazebo/OGRE 2 rendering stack —
measured at 30+ additional packages and roughly 3.7 GB in the built image.

That is a direct violation of the project's rule that no desktop-OpenGL
software may run on the target: the AM69 GPU exposes only OpenGL ES 3.2 and
Vulkan 1.2, so OGRE 2 has no business on the module. `--no-install-recommends`
does not help here — these are hard `Depends`, not `Recommends`.

## What changed from upstream

Exactly two kinds of edit, both path re-rooting, applied to all four files:

1. `get_package_share_directory('nav2_bringup')` → `('demo_navigation')`.
2. In `bringup_launch.py`, `launch_dir` now points at
   `<demo_navigation>/launch/nav2_vendored/`, so the three sibling includes
   resolve here instead of in the (absent) upstream package.

A consequence of (1): the `params_file` default was
`<pkg>/params/nav2_params.yaml`; this package keeps parameters in `config/`,
so the four defaults now read `<pkg>/config/nav2_params.yaml`. In practice
`navigation.launch.py` always passes `params_file` explicitly, so this default
is a fallback, not the live path.

**No behavioral logic was modified.** No node, parameter, remap, lifecycle
transition, or composition structure was touched.

## Updating these files

They are a snapshot and will drift from upstream. When bumping the Nav2
version:

1. Re-copy the four files from the new `nav2_bringup`.
2. Re-apply the two path edits above (`grep -n nav2_bringup` finds them all).
3. Diff against the previous vendored copy to see what upstream changed.

## What is still installed from apt

`ros-jazzy-navigation2` — the servers themselves (AMCL, controller, planner,
BT navigator, lifecycle manager, costmap, map server, …). That metapackage
does pull `nav2-rviz-plugins`, a small RViz *plugin* library that does not
drag in OGRE or Gazebo. The heavy graphical dependency chain came from the
`nav2-bringup` simulation dependencies, not from the Nav2 servers.
