# ADR 0013: Explorer watchdog instead of further MPPI tuning

- **Status:** Accepted

## Context

The quadruped zigzags around straight plans. Diagnostic rounds showed the
cause is MPPI oscillation, not a bent plan or gait overshoot. Two isolated A/B
experiments on the `PathAlignCritic` weights were neutral or negative on the
AM69.

## Decision

The MPPI oscillation is treated as mitigated but not eliminated. The
consequences are handled in `maze_explorer`:

- A movement watchdog, armed when a goal is accepted, cancels goals that have
  genuinely stalled.
- A spin-based sweep recovery handles the case where no frontier is left.
- The map sequence is tracked by content hash, so a republished identical map
  cannot release a suppression.
- The frontier route penalty grows with `sqrt(route_m)` instead of linearly.
  The linear penalty kept the frontiers towards the exit from ever winning.

## Consequences

- R16 on the AM69 showed the smoothest motion of any round. Both watchdog
  firings were confirmed as real stalls, with no false positives.
- The goal completion rate did not clearly improve. The sweep recovery has not
  yet triggered in HIL.
