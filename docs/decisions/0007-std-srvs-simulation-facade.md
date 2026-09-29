# ADR 0007: The browser never calls Gazebo interfaces

- **Status:** Accepted

## Context

The first simulation controls in the cockpit called
`ros_gz_interfaces/srv/ControlWorld` directly through rosbridge. They failed
at runtime because the `cockpit` container does not ship `ros_gz_interfaces`.
On a real robot there would be no Gazebo to call at all.

## Decision

`sim_control_relay` runs on the simulation side and exposes
`/demo/sim/{play,pause,reset}` as `std_srvs/srv/Trigger`. It translates those
calls to Gazebo services internally. `scene_view_controller` uses the same
pattern for the scene cameras.

## Consequences

- The cockpit only depends on standard ROS interfaces.
- A guard test (`test_browser_never_speaks_gazebo_interfaces`) enforces this.
