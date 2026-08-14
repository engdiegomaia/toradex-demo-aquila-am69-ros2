# Vendored Nav2 launch files

Verbatim copies of four launch files from `nav2_bringup`, with **only** the
package-root paths re-rooted. Copyright headers are intact.

| File | Origin |
| --- | --- |
| `bringup_launch.py` | `nav2_bringup/launch/bringup_launch.py` |
| `localization_launch.py` | `nav2_bringup/launch/localization_launch.py` |
| `navigation_launch.py` | `nav2_bringup/launch/navigation_launch.py` |
| `slam_launch.py` | `nav2_bringup/launch/slam_launch.py` |

**Source:** `ros-jazzy-nav2-bringup`, ROS 2 Jazzy, Ubuntu 24.04 arm64/amd64 apt
packages, extracted 14/08/2026.
**License:** Apache-2.0 (Copyright 2018 Intel Corporation and Nav2 contributors),
the same license this project uses.

## Why these are vendored (ML3.5 F1)

`demo_navigation/navigation.launch.py` delegates to `bringup_launch.py` on
purpose: the lifecycle-manager wiring and node ordering are what upstream
maintains, and hand-duplicating them here would rot. That decision stands.

What could not stand is installing the `ros-jazzy-nav2-bringup` **package** into
the `nav` container image. `nav` is one of the two images that travels to the
Aquila AM69, and `nav2-bringup` hard-depends on:

    ros-jazzy-nav2-minimal-tb3-sim
    ros-jazzy-nav2-minimal-tb4-sim
    ros-jazzy-ros-gz-sim
    ros-jazzy-ros-gz-bridge
    ros-jazzy-navigation2   (which pulls ros-jazzy-nav2-rviz-plugins)

Those pull the entire Gazebo stack — measured in the built image: `libogre-1.9`,
`ros-jazzy-gz-ogre-next-vendor`, `gz-rendering`, `gz-gui`, `gz-sim` and 30+ more
packages, for a 3.7 GB image.

That is a direct violation of **CLAUDE.md rule 1**: no desktop-OpenGL software on
the target. The AM69 exposes only OpenGL ES 3.2 and Vulkan 1.2, so OGRE 2 has no
business on the module — and it is dead weight on a Torizon data partition
besides (`guia-ml35-docker.md` §4).

`--no-install-recommends` does not help: these are hard `Depends`, not
`Recommends`.

## What changed from upstream

Exactly two kinds of edit, both path re-rooting:

1. `get_package_share_directory('nav2_bringup')` → `('demo_navigation')`,
   in all four files.
2. In `bringup_launch.py`, `launch_dir` now points at
   `<demo_navigation>/launch/nav2_vendored/` so the three sibling includes
   resolve here instead of in the (absent) upstream package.

Plus a consequence of (1): the `params_file` default was
`<pkg>/params/nav2_params.yaml`; this package keeps its parameters in `config/`,
so those four defaults now read `<pkg>/config/nav2_params.yaml`. In practice
`navigation.launch.py` always passes `params_file` explicitly, so the default is
a fallback, not the live path.

**No behavioural logic was modified.** No node, parameter, remap, lifecycle
transition or composition was touched.

## Updating these files

They are a snapshot and will drift from upstream. When bumping Nav2:

1. Re-copy the four files from the new `nav2_bringup`.
2. Re-apply the two path edits above (`grep -n nav2_bringup` finds them all).
3. Diff against the previous vendored copy to see what upstream changed.

## What is still installed from apt

`ros-jazzy-navigation2` — the servers themselves (AMCL, controller, planner,
BT navigator, lifecycle manager, costmap, map server, …). That metapackage does
pull `nav2-rviz-plugins`, which is a small RViz *plugin* library and does **not**
drag OGRE or Gazebo. The heavy graphical chain came from the `nav2-bringup`
simulation dependencies, not from the servers.
