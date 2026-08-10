"""
Robot description tests (ML2 Definition of Done).

Proves the xacro expands, the URDF parses, and the TF tree required by
.ai/AGENTS.md §5.3 exists with a single root and no degenerate inertia.

These run without a ROS graph: xacro expansion plus XML parsing only.
"""

import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import pytest

# Frames the project contract depends on. Nav2 uses base_footprint as
# robot_base_frame; sensor frames must match the gz_frame_id set in _sensors.xacro,
# otherwise scans and images arrive with a frame_id absent from the TF tree.
REQUIRED_LINKS = {
    'base_footprint',
    'base_link',
    'laser_frame',
    'camera_link',
    'left_wheel_link',
    'right_wheel_link',
    'caster_wheel_link',
}

# The DiffDrive plugin in demo_robot.urdf.xacro references these joint names
# verbatim. They must stay continuous, or locomotion silently breaks in ML3.
REQUIRED_CONTINUOUS_JOINTS = {'left_wheel_joint', 'right_wheel_joint'}

EXPECTED_ROOT_LINK = 'base_footprint'


def _expand_xacro() -> str:
    xacro_path = (
        get_package_share_directory('demo_description') + '/urdf/demo_robot.urdf.xacro'
    )
    result = subprocess.run(
        ['xacro', xacro_path],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f'xacro failed:\n{result.stderr}'
    return result.stdout


@pytest.fixture(scope='module')
def urdf_root() -> ET.Element:
    return ET.fromstring(_expand_xacro())


def test_xacro_expands_and_parses(urdf_root: ET.Element) -> None:
    assert urdf_root.tag == 'robot'
    assert urdf_root.get('name') == 'demo_robot'


def test_required_links_exist(urdf_root: ET.Element) -> None:
    links = {link.get('name') for link in urdf_root.findall('link')}
    missing = REQUIRED_LINKS - links
    assert not missing, f'URDF is missing required links: {sorted(missing)}'


def test_tree_has_single_root(urdf_root: ET.Element) -> None:
    links = {link.get('name') for link in urdf_root.findall('link')}
    children = {
        joint.find('child').get('link') for joint in urdf_root.findall('joint')
    }
    roots = links - children
    assert roots == {EXPECTED_ROOT_LINK}, \
        f'expected exactly one root ({EXPECTED_ROOT_LINK}), got {sorted(roots)}'


def test_wheel_joints_are_continuous(urdf_root: ET.Element) -> None:
    joint_types = {
        joint.get('name'): joint.get('type') for joint in urdf_root.findall('joint')
    }
    for name in REQUIRED_CONTINUOUS_JOINTS:
        assert name in joint_types, f'missing wheel joint {name}'
        assert joint_types[name] == 'continuous', \
            f'{name} must be continuous, got {joint_types[name]}'


def test_sensors_are_fixed_to_base_link(urdf_root: ET.Element) -> None:
    parents = {
        joint.find('child').get('link'):
            (joint.find('parent').get('link'), joint.get('type'))
        for joint in urdf_root.findall('joint')
    }
    for frame in ('laser_frame', 'camera_link'):
        parent, joint_type = parents[frame]
        assert parent == 'base_link', f'{frame} must hang off base_link, got {parent}'
        assert joint_type == 'fixed', f'{frame} joint must be fixed, got {joint_type}'


def test_base_footprint_is_parent_of_base_link(urdf_root: ET.Element) -> None:
    """base_footprint is the ground projection and therefore base_link's parent."""
    joint = next(
        j for j in urdf_root.findall('joint')
        if j.find('child').get('link') == 'base_link'
    )
    assert joint.find('parent').get('link') == 'base_footprint'
    assert joint.get('type') == 'fixed'


def test_no_degenerate_inertia(urdf_root: ET.Element) -> None:
    """
    Every colliding link needs positive mass and inertia.

    Zero or missing inertia is the classic first-run gz sim failure: the model
    sinks through the floor or explodes on spawn.
    """
    for link in urdf_root.findall('link'):
        name = link.get('name')
        inertial = link.find('inertial')
        if inertial is None:
            assert link.find('collision') is None, \
                f'{name} has collision geometry but no inertial block'
            continue

        mass = float(inertial.find('mass').get('value'))
        assert mass > 0.0, f'{name} has non-positive mass {mass}'

        inertia = inertial.find('inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            value = float(inertia.get(axis))
            assert value > 0.0, f'{name} has non-positive {axis}={value}'


def test_wheel_separation_matches_diff_drive_plugin(urdf_root: ET.Element) -> None:
    """
    The DiffDrive plugin's wheel_separation must equal the actual track width.

    If these drift apart, Gazebo reports odometry that does not match the
    physical motion and Nav2's localization degrades in a way that is very hard
    to trace back to the URDF.
    """
    plugin = next(
        p for gz in urdf_root.findall('gazebo')
        for p in gz.findall('plugin')
        if p.get('name') == 'gz::sim::systems::DiffDrive'
    )
    declared = float(plugin.find('wheel_separation').text)

    origins = {}
    for joint in urdf_root.findall('joint'):
        if joint.get('name') in REQUIRED_CONTINUOUS_JOINTS:
            origins[joint.get('name')] = float(
                joint.find('origin').get('xyz').split()[1]
            )

    actual = abs(origins['left_wheel_joint'] - origins['right_wheel_joint'])
    assert actual == pytest.approx(declared, abs=1e-6), \
        f'DiffDrive wheel_separation={declared} but wheels are {actual} apart'
