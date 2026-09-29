# ADR 0010: Compressed camera transport across the link

- **Status:** Accepted

## Context

The raw camera stream (640×480 @ 10 Hz, about 74 Mbit/s) competed with Nav2
for link bandwidth and for module CPU.

## Decision

The host republishes the camera as `image_transport` compressed. The module
decompresses it just before `demo_perception`, which still consumes a plain
`sensor_msgs/Image`. The raw contract topic keeps being published on the host.

## Consequences

- Measured on the AM69: 82× less traffic on the link and module CPU from 711 %
  to 600 %.
- Navigation speed did **not** improve. The cost of running perception at all
  dominates, not the transport format.
- The topic contract ([ADR 0005](0005-perception-topic-contract.md)) is
  unchanged.
