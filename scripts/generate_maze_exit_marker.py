#!/usr/bin/env python3
"""Generate the printed AprilTag texture for the maze exit marker.

Deterministic: same dictionary, id and pixel size in, same PNG bytes out.
Not downloaded from anywhere — regenerate with this script if the tag ever
needs to change, instead of hand-editing the PNG.

The id, dictionary and physical size here MUST match the detector's defaults
in ``demo_perception/maze_exit_detector.py``
(``FIDUCIAL_DICTIONARIES['DICT_APRILTAG_36h11']``, ``fiducial_id=0``,
``FIDUCIAL_SIZE_M_DEFAULT=0.64``) — the two are not wired together
programmatically, so a change on one side must be mirrored on the other.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11
MARKER_ID = 0
TAG_PIXELS = 512
# Extra white quiet zone around the tag core, as a fraction of TAG_PIXELS.
# Real AprilTag detectors rely on this margin; the dictionary's own border
# bits are not a substitute for it.
MARGIN_FRACTION = 0.15

DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[1]
    / 'ros2_ws/src/demo_simulation/worlds/maze_exit_tag.png'
)


def render_tag(
    marker_id: int = MARKER_ID,
    tag_pixels: int = TAG_PIXELS,
    margin_fraction: float = MARGIN_FRACTION,
) -> np.ndarray:
    """Return the grayscale tag image, tag core plus a white quiet zone."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    core = cv2.aruco.drawMarker(dictionary, marker_id, tag_pixels)
    margin = round(tag_pixels * margin_fraction)
    return cv2.copyMakeBorder(
        core, margin, margin, margin, margin,
        cv2.BORDER_CONSTANT, value=255)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    image = render_tag()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.out), image):
        raise SystemExit(f'failed to write {args.out}')
    print(f'wrote {args.out} ({image.shape[1]}x{image.shape[0]}px, '
          f'DICT_APRILTAG_36h11 id={MARKER_ID})')


if __name__ == '__main__':
    main()
