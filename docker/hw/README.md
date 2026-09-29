# `hw` — real A1/Go2 hardware driver container

**Empty on purpose. Out of scope for the current milestone.**

This directory exists so a known problem has a home before it becomes a
surprise during hardware bring-up.

## The registered invariant collision

`legubiao/quadruped_ros2_control`, the locomotion stack this project vendors
(see [ADR 0004](../../docs/decisions/0004-quadruped-control-stack.md)),
documents that **CycloneDDS conflicts with `unitree_sdk2`** and recommends
Fast DDS instead.

The project's non-negotiable rule 2 is `rmw_cyclonedds_cpp` **always**, baked
into the base image as `ENV` and never swapped at runtime.

These two constraints cannot both hold in a container that talks to a
physical robot over `unitree_sdk2`.

## Why this does not block simulation-based milestones

`unitree_sdk2` only enters the picture with a physical robot. Every
simulation-based phase drives a simulated plant through Gazebo, where the SDK
is absent and there is no collision. The decision is genuinely deferred, not
ignored.

## What has to be decided before this container gets content

- Whether the SDK conflict is real on ROS 2 Jazzy with the current
  `unitree_sdk2`, or a stale note in the upstream README. Verify against the
  SDK source, not the README — the same standard the rest of this project
  applies to upstream claims.
- If real: whether `hw` becomes the single Fast DDS exception (a documented
  bridge at the container boundary), or whether the SDK is isolated behind a
  non-ROS process that speaks to the rest of the stack over the topic
  contract.

Either path is an ADR (see `docs/decisions/README.md`), not a code change to
make in passing.

## What runs here eventually

The real robot driver: joint commands out, joint state and IMU in, publishing
the same contract the simulated plant publishes today — `/demo/odom`,
`/demo/scan`, `/demo/cmd_vel`. Nothing above the plant may learn that the
robot became physical (see the topic-contract rule in the repository root
`CLAUDE.md`).

Architecture: **arm64 only**, module only. No physical-hardware validation
has been performed; do not treat anything in this directory as
hardware-validated.
