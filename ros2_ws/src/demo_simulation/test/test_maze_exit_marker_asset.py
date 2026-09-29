"""
Structural checks for the maze exit fiducial marker asset.

The tag is a rendering add-on next to the existing magenta panel (see
`maze_exit_marker` in `quadruped_maze11.sdf`): the panel's own pose and size
are the escape-detection contract's business (`maze_escape_validator.py`)
and must not move. These tests only guard that the tag decal matches what
`demo_perception/maze_exit_detector.py` expects by default (dictionary,
id, physical size) and that its texture ships with the package.
"""

from pathlib import Path
import re

import cv2
import pytest


WORLD = Path(__file__).parents[1] / 'worlds' / 'quadruped_maze11.sdf'
TAG_PNG = Path(__file__).parents[1] / 'worlds' / 'maze_exit_tag.png'

# Must match demo_perception/maze_exit_detector.py's declared parameter
# defaults (fiducial_dictionary, fiducial_id, fiducial_size_m).
EXPECTED_DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11
EXPECTED_ID = 0
EXPECTED_SIZE_M = 0.64
# The panel itself, unchanged since before the fiducial existed.
PANEL_POSE = ('-4.90', '-2.60', '0.60', '0', '0', '0')


def _world_body() -> str:
    text = WORLD.read_text(encoding='utf-8')
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


def _exit_marker_model() -> str:
    body = _world_body()
    match = re.search(
        r'<model name="maze_exit_marker">.*?</model>', body, re.DOTALL)
    assert match, 'modelo "maze_exit_marker" nao encontrado em ' + WORLD.name
    return match.group(0)


def test_exit_marker_pose_is_unchanged_by_the_fiducial_addition() -> None:
    """The escape-validator's OPENING_X/BOUNDARY_Y math depends on this pose."""
    model = _exit_marker_model()
    pose = re.search(r'<pose>\s*([^<]+?)\s*</pose>', model)
    assert pose, 'maze_exit_marker sem <pose>'
    assert tuple(pose.group(1).split()) == PANEL_POSE


def test_magenta_panel_visual_is_still_present() -> None:
    """The fiducial is additive: the magenta fallback panel must survive."""
    model = _exit_marker_model()
    assert re.search(r'<visual name="panel">', model)
    assert '1.0 0.0 1.0 1.0' in model  # ambient/diffuse magenta, unchanged


def test_tag_visual_references_the_installed_texture() -> None:
    model = _exit_marker_model()
    tag = re.search(
        r'<visual name="tag">.*?</visual>', model, re.DOTALL)
    assert tag, 'visual "tag" nao encontrada em maze_exit_marker'
    assert '<albedo_map>maze_exit_tag.png</albedo_map>' in tag.group(0)

    size = re.search(r'<size>\s*([^<]+?)\s*</size>', tag.group(0))
    assert size, 'visual "tag" sem <box><size>'
    width, _thickness, height = (float(v) for v in size.group(1).split())
    assert width == pytest.approx(EXPECTED_SIZE_M)
    assert height == pytest.approx(EXPECTED_SIZE_M)


def test_tag_texture_is_shipped_next_to_the_world_not_downloaded() -> None:
    assert TAG_PNG.is_file(), (
        f'{TAG_PNG} ausente -- rode tools/maze/generate_maze_exit_marker.py')


def test_tag_texture_decodes_to_the_id_the_detector_expects() -> None:
    """
    Decode the shipped PNG and check it carries the id the detector expects.

    If this fails, either the PNG was hand-edited or the detector's defaults
    drifted -- `demo_perception/maze_exit_detector.py`'s
    `fiducial_id`/`fiducial_dictionary` parameters must match this asset.
    """
    image = cv2.imread(str(TAG_PNG), cv2.IMREAD_GRAYSCALE)
    assert image is not None, f'{TAG_PNG} nao decodifica como imagem'

    dictionary = cv2.aruco.getPredefinedDictionary(EXPECTED_DICTIONARY)
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, _rejected = cv2.aruco.detectMarkers(
        image, dictionary, parameters=params)

    assert ids is not None and len(ids) == 1, (
        f'esperava exatamente 1 tag detectada no PNG puro, achou {ids}')
    assert int(ids[0][0]) == EXPECTED_ID
    assert len(corners) == 1
