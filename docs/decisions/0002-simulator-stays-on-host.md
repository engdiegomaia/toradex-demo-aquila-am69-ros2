# ADR 0002: Gazebo and RViz2 never run on the module

- **Status:** Accepted
- **Scope:** architecture-wide

## Context

The Aquila AM69 GPU exposes OpenGL ES 3.2 and Vulkan 1.2 only. Gazebo
Harmonic (OGRE 2), RViz2 and anything else built on desktop OpenGL cannot run
there.

## Decision

The simulator, the ROS–Gazebo bridge and RViz2 run only on the x86 host. Only
components that would also run on a real robot, namely Nav2, perception and
the cockpit backend, are deployed to the module. Containers are split by
"what changes when the simulation is replaced by hardware": `sim` (and a
future `hw`) are swappable, while `nav` and `perception` stay the same.

## Consequences

- `compose.module.yml` has no `sim`, `viz`, X11 socket or `/dev/dri` mapping.
- `rosdep` and apt can pull rendering libraries transitively. Nothing fails at
  build time when that happens. `scripts/module.sh build` therefore scans every
  module image for OGRE, gz-rendering, gz-sim, gz-gui and RViz libraries and
  fails the build if it finds any.
- "Start the simulation on the target" is not achievable. The cockpit can only
  *control* the host's simulation.
