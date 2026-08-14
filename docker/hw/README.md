# `hw` — real A1 hardware driver container

**Empty on purpose. Out of scope for ML3.5.**

This directory exists from F1 so that a known problem has a home before it
becomes a surprise during hardware bring-up.

## The registered invariant collision

`legubiao/quadruped_ros2_control`, the locomotion base chosen for ML3.5,
documents that **CycloneDDS conflicts with `unitree_sdk2`** and recommends Fast
DDS instead.

The project's non-negotiable rule 2 is `rmw_cyclonedds_cpp` **always**, baked
into the base image as `ENV` and never swapped.

These cannot both hold in a container that talks to a physical A1.

## Why it does not block ML3.5

`unitree_sdk2` only enters with a physical A1. Every ML3.5 phase drives a
simulated robot through Gazebo, where the SDK is absent and there is no
collision. The decision is genuinely deferred, not ignored.

## What has to be decided before this container gets content

- Whether the SDK conflict is real on Jazzy + `unitree_sdk2` current, or a stale
  note in the upstream README. **Verify against the SDK source, not the README** —
  the same standard the rest of this milestone applies to upstream claims.
- If real: whether `hw` becomes the single Fast DDS exception (a documented
  bridge at the container boundary), or whether the SDK is isolated behind a
  non-ROS process that speaks to the rest of the stack over the topic contract.

Either path is an ADR, not a code change to make in passing.

## What runs here eventually

The real A1 driver: joint commands out, joint state and IMU in, publishing the
same contract the simulated plant publishes today — `/demo/odom`, `/demo/scan`,
`/demo/cmd_vel`. Nothing above the plant may learn that the robot became
physical (CLAUDE.md rule 6).

Architecture: **arm64 only**, module only.
