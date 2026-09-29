# ADR 0008: Simulation reset teleports the robot

- **Status:** Accepted

## Context

`ControlWorld.reset.all` returns the world to its original SDF. That SDF does
not contain the robot or the scene cameras, which are spawned later. After a
reset the clock kept running and the sensors kept publishing from an orphaned
model, and no log reported it. Recovery required restarting the `sim`
container.

## Decision

`/demo/sim/reset` teleports the robot to the scenario's spawn pose through
`/demo/sim/set_entity_pose` and never touches the world or the clock. The
reset runs `/demo/gait/hold` → teleport → `/demo/gait/resume`, so that the gait
controller's captured body reference and the velocity Gazebo preserves on a
teleport cannot make the robot collapse or drift. Clearing navigation state is
a separate service, `/demo/nav/reset`.

## Consequences

- The reset recovers from a teleport in the middle of a gait and from a full
  tip-over.
- Recreating the `nav` container must always happen **after** a simulation
  reset, never before. Otherwise SLAM anchors its map to a stale pose.
  `scripts/module.sh up` refuses to start when the robot is far from spawn.
