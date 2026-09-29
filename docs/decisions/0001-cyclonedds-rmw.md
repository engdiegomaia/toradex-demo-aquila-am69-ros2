# ADR 0001: CycloneDDS is the only RMW

- **Status:** Accepted
- **Scope:** host and module

## Context

The stack spans several containers on two machines. With the default RMW
(Fast DDS) inter-container discovery failed. Discovery across the host–module
link also has to work without multicast, which is unreliable over Wi-Fi and
across Docker bridges.

## Decision

`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is baked into the base image with
`ENV` and is never set or changed at runtime. CycloneDDS runs with
`AllowMulticast=false`, an explicit `127.0.0.1` peer for same-machine
discovery, and an explicit unicast peer for the other machine. The peer
configuration is rendered at deploy time by `scripts/module.sh` from the
committed templates in `docker/cyclonedds/`.

## Consequences

- Every networking fix in the project is CycloneDDS-specific. Replacing the
  RMW means re-validating discovery from scratch.
- `unitree_sdk2`, which a future physical-robot driver would need, recommends
  Fast DDS. That conflict is deliberately deferred to the (absent) `hw`
  service. See `docker/hw/README.md`.
