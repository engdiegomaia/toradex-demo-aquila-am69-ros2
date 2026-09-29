# ADR 0014: `route_server` rerouting disabled

- **Status:** Accepted

## Context

A full lifecycle `RESET` → `STARTUP` of the Nav2 stack crashed
`nav2_container` with SIGSEGV. The crash happened every time, inside
`route_server` while it configured its rerouting service. Cycling the
lifecycle of a single managed node leads to the same crash.

## Decision

`route_server.operations` is restricted to `["AdjustSpeedLimit"]`. The
cockpit's `/demo/nav/reset` never goes through `CONFIGURE`: it cancels the
goal, clears the costmaps while the servers are active, then pauses and
resumes. To fully restart navigation, recreate the `nav` container.

## Consequences

- The crash is probably an upstream Nav2 bug. It has not been reported yet.
- Localization is kept out of every reset path.
