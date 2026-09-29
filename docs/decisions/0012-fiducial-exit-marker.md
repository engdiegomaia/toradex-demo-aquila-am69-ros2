# ADR 0012: AprilTag fiducial as primary exit detector

- **Status:** Accepted

## Context

The original exit detector estimated range from the width of a magenta panel.
Under partial occlusion (a panel seen through a narrow opening) it read the
panel as farther away than it was. The mean error grew from 0.41 m at 3–4 m to
3.08 m beyond 6 m. A width-based estimator cannot fix this in principle.

## Decision

`maze_exit_detector` has a `detector_backend` parameter. The default is
`fiducial`: AprilTag `36h11`, id 0, through OpenCV's `cv2.aruco`. `magenta`
remains available as a fallback, and only one backend publishes at a time. The
detector fails closed: it publishes no detection rather than a confident wrong
range.

## Consequences

- Reliability depends on range. On the AM69, 14/20 frames were detected at
  about 2.5 m and 1/21 at about 3.6 m.
- The dictionary, id and size are code defaults, not read from the scenario,
  so perception still knows nothing about the maze.
