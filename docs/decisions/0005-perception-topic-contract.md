# ADR 0005: Perception talks only through a fixed topic contract

- **Status:** Accepted

## Context

Perception is a deterministic stub today. The target is inference on the AM69
accelerators (TIDL). That swap must not ripple through the rest of the system.

## Decision

`demo_perception` consumes `/demo/camera/image_raw` (`sensor_msgs/Image`) and
publishes `/demo/perception/detections` (`vision_msgs/Detection2DArray`).
Nothing else is part of its interface. It never knows where the image comes
from (simulation, USB camera or rosbag). No consumer knows whether the
detections come from the stub or from real inference. Detections feed a Nav2
costmap layer as well as the HMI.

## Consequences

- Swapping in TIDL means swapping a container, not changing an interface.
- Contract tests keep perception and the frontier explorer from knowing the
  maze.
- Perception gets its own container from the start, because that container is
  the unit of an over-the-air update.
