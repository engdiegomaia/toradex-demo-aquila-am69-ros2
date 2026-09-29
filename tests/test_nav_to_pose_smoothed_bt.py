"""
Lock the blackboard collision that produces `[follow_path] Aborting handle` at
~1 Hz throughout navigation (see docs/ml35/proximos-passos-navegacao.md).

`SmoothPath` and `FollowPath` live as siblings inside the same
`PipelineSequence`. While `FollowPath` returns RUNNING, the `PipelineSequence`
itself re-ticks the preceding siblings -- including the `RateController` that
recomputes and smooths the path. If `SmoothPath` writes its result to the SAME
key that `FollowPath` reads (`{path}`), every re-tick looks like a new goal to
`FollowPath`, which aborts the handle in progress and starts over. Confirmed by
a Nav2 maintainer in ros-navigation/navigation2#5817: "we expect users to remap
the smoothed path to a different blackboard variable".

This test needs neither ROS nor a simulator: it is a structural check of the
XML.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

BT_FILE = (
    Path(__file__).resolve().parents[1]
    / 'ros2_ws' / 'src' / 'demo_navigation' / 'behavior_trees'
    / 'nav_to_pose_smoothed.xml'
)


def _find_one(root: ET.Element, tag: str) -> ET.Element:
    matches = root.findall(f'.//{tag}')
    assert len(matches) == 1, f'expected exactly one <{tag}>, found {len(matches)}'
    return matches[0]


def test_smooth_path_does_not_overwrite_its_own_input():
    """
    `unsmoothed_path` and `smoothed_path` on the SAME node must be different
    keys. Reusing the same key is exactly the pattern reproduced in
    navigation2#5817.
    """
    root = ET.parse(BT_FILE).getroot()
    smooth = _find_one(root, 'SmoothPath')
    assert smooth.get('unsmoothed_path') != smooth.get('smoothed_path'), (
        "SmoothPath reuses the same key for input and output -- that is the "
        "pattern that causes 'Aborting handle' at ~1 Hz (navigation2#5817)")


def test_follow_path_consumes_the_smoothed_path():
    """
    The point of `SmoothPath` is to feed `FollowPath` the already smoothed path.
    If the keys do not match, `FollowPath` goes back to following NavFn's
    staircase path, the defect `nav_to_pose_smoothed.xml` exists to avoid.
    """
    root = ET.parse(BT_FILE).getroot()
    smooth = _find_one(root, 'SmoothPath')
    follow = _find_one(root, 'FollowPath')
    assert follow.get('path') == smooth.get('smoothed_path')
