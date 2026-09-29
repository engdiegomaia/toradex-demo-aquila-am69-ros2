"""
Invariants of the compressed camera transport between host and module.

Structural tests: they read the launch files with `ast`, they do not bring up
ROS. Matching a raw string would be fragile here specifically -- the first
version of these tests passed while the launch was broken, because it
asserted the positional shape that was the actual defect.

What they protect cost bench time, not opinion:
docs/results/ml35-f5-ethernet0-repeticao.md.
"""

from __future__ import annotations

import ast
from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / 'launch'
CONTRACT_TOPIC = '/demo/camera/image_raw'
LOCAL_TOPIC = '/demo/perception/image_in'


def _republish_nodes(launch_file: str) -> dict[str, dict]:
    """Extract {name: {params, remaps}} from each Node(executable='republish')."""
    tree = ast.parse((LAUNCH / launch_file).read_text(encoding='utf-8'))
    found: dict[str, dict] = {}

    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        executable = kw.get('executable')
        if not isinstance(executable, ast.Constant) or executable.value != 'republish':
            continue

        params: dict[str, str] = {}
        for entry in getattr(kw.get('parameters'), 'elts', []):
            if not isinstance(entry, ast.Dict):
                continue
            for key, value in zip(entry.keys, entry.values):
                if isinstance(key, ast.Constant) and isinstance(value, ast.Constant):
                    params[key.value] = value.value

        remaps = {}
        for pair in getattr(kw.get('remappings'), 'elts', []):
            if isinstance(pair, ast.Tuple) and len(pair.elts) == 2:
                src, dst = pair.elts
                if isinstance(src, ast.Constant) and isinstance(dst, ast.Constant):
                    remaps[src.value] = dst.value

        name = kw['name'].value
        found[name] = {'params': params, 'remaps': remaps}

    return found


HOST = _republish_nodes('sim.launch.py')
MODULE = _republish_nodes('perception.launch.py')


def _expected_remap_key(side: str, transport: str) -> str:
    """
    Name the topic as `<side>/<transport>`, except for raw.

    `raw` is the default transport and uses the base topic with no suffix.
    Any other transport adds the suffix, and the remap must match the FULL
    name.
    """
    return side if transport == 'raw' else f'{side}/{transport}'


def _effective_topic(node: dict, side: str) -> str:
    transport = node['params'][f'{side}_transport']
    return node['remaps'][_expected_remap_key(side, transport)]


def test_both_sides_declare_a_republish_node() -> None:
    assert 'camera_compressor' in HOST
    assert 'camera_decompressor' in MODULE


def test_every_republish_sets_both_transports_explicitly() -> None:
    """
    A missing `out_transport` does not raise an error -- it gives a mute node, or a loop.

    Written as arguments=['raw', 'compressed'], Jazzy reads the first as
    in_transport and leaves out_transport empty, logging
    "The 'out_transport' parameter is set to:" with a blank value. No error
    line. That is how the contract camera ended up at 118 Hz.
    """
    for name, node in {**HOST, **MODULE}.items():
        for key in ('in_transport', 'out_transport'):
            assert node['params'].get(key), \
                f'{name}: {key} must be an explicit, non-empty parameter'


def test_remap_keys_carry_the_transport_suffix() -> None:
    """
    Remapping `out` when the topic is named `out/compressed` does NOT match.

    The rule is ignored silently: the node comes up, logs the right
    transports, subscribes to the input, and publishes on `/out/compressed`
    at the root -- a topic nobody looks for. Measured on 25/08/2026; only
    `ros2 node info` gave it away.
    """
    for name, node in {**HOST, **MODULE}.items():
        for side in ('in', 'out'):
            expected = _expected_remap_key(side, node['params'][f'{side}_transport'])
            assert expected in node['remaps'], (
                f'{name}: missing remap for {expected!r}; '
                f'the keys present are {sorted(node["remaps"])}'
            )


def test_no_republish_can_feed_itself() -> None:
    """
    Republishing must never feed itself.

    Publishing on the topic it subscribes to does not raise an error -- it
    gives feedback.

    Measured: publisher count 2 on the contract topic, and the camera at
    118 Hz instead of 10 Hz, with the module's detection_stub receiving the
    stream inflated by the wire.
    """
    for name, node in {**HOST, **MODULE}.items():
        assert _effective_topic(node, 'in') != _effective_topic(node, 'out'), \
            f'{name}: publishes on the same topic it subscribes to'


def test_the_two_sides_agree_on_the_wire_topic() -> None:
    """
    What the host publishes must be exactly what the module subscribes to.

    They are different files, on different machines, and nothing at runtime
    complains if they diverge: the decompressor simply never receives
    anything, and perception dies silently.
    """
    assert _effective_topic(HOST['camera_compressor'], 'out') == \
        _effective_topic(MODULE['camera_decompressor'], 'in')


def test_module_output_never_reuses_the_contract_topic_name() -> None:
    """Two publishers on one topic raise no error; the source just alternates."""
    assert _effective_topic(MODULE['camera_decompressor'], 'out') == LOCAL_TOPIC
    assert _effective_topic(HOST['camera_compressor'], 'in') == CONTRACT_TOPIC


def test_perception_is_rewired_by_remap_and_not_by_editing_the_node() -> None:
    """
    Rule 6: demo_perception does not know the origin of the frame.

    The rewiring lives in the launch file. If it turns into an edit in
    detection_stub.py, the node starts to know the transport topology, and
    swapping in TIDL stops being a container swap.
    """
    perception = (LAUNCH / 'perception.launch.py').read_text(encoding='utf-8')
    assert f"SetRemap(src='{CONTRACT_TOPIC}', dst='{LOCAL_TOPIC}')" in perception

    stub = (LAUNCH.parents[1] / 'demo_perception' / 'demo_perception'
            / 'detection_stub.py').read_text(encoding='utf-8')
    assert 'CompressedImage' not in stub
    assert f"IMAGE_TOPIC = '{CONTRACT_TOPIC}'" in stub


def test_decompressor_stays_outside_the_remap_scope() -> None:
    """Inside the SetRemap scope, the decompressor's `in` would become itself."""
    perception = (LAUNCH / 'perception.launch.py').read_text(encoding='utf-8')
    group_start = perception.index('perception_group = GroupAction([')
    assert perception.index('camera_decompressor = Node(') < group_start
    group_body = perception[group_start:perception.index('])', group_start)]
    assert 'camera_decompressor' not in group_body
