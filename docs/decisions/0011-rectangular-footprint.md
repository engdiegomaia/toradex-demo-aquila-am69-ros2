# ADR 0011: Rectangular footprint instead of `robot_radius`

- **Status:** Accepted

## Context

A 0.38 m circle circumscribed around the Go2 trunk (0.70 × 0.31 m) wasted
about 23 cm of corridor per side in 1.20 m maze corridors. The robot's own
costmap cell reached the lethal band, so the planner refused to plan from the
start pose.

## Decision

Both costmaps use the trunk rectangle (`[±0.37, ±0.18]` m) as the footprint.
This was promoted to the default Nav2 parameters after a controlled A/B on
the AM69.

## Consequences

- The robot's own-cell cost now peaks at 165 (it used to reach 243; lethal is
  253). Exploration now runs its full 600 s budget.
- `nav2_params_go2_footprint.yaml` is kept identical to the default for older
  campaign commands. A test checks that the two files stay identical.
