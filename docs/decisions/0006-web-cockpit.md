# ADR 0006: Web cockpit instead of X11 window embedding

- **Status:** Accepted (supersedes the PyQt5/X11 embedding cockpit)

## Context

The first operator console tried to embed the Gazebo, RViz2 and image-viewer
windows into one Qt window. RViz2 and `rqt_image_view` do not implement
XEmbed, and forcing reparenting meant disabling the desktop compositor. Such a
console could also never run on the module (see [ADR 0002](0002-simulator-stays-on-host.md)).

## Decision

The cockpit is rendered from ROS data instead of captured windows:

- `rosbridge_server` and `web_video_server` run in the `cockpit` container.
- A static HTML/CSS/ES-module bundle in `hmi/` is served by nginx.
- There is no build step and there are no npm dependencies. A minimal
  rosbridge client replaces `roslibjs`.

## Consequences

- The same bundle serves the host cockpit today and a kiosk browser on the
  module later, without rework.
- Gazebo's free camera orbit and RViz2's runtime layer toggles are lost. Scene
  cameras controlled through ROS replace them.
- `scripts/cockpit.py` (the desktop embedding tool) is kept only as an
  optional developer convenience.
