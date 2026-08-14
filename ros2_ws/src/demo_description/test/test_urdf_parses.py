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

# Frames the project contract depends on.
#
# These are the UPSTREAM TurtleBot 4 names (nav2_minimal_tb4_description), not
# the ones the old hand-built model used. The rename happened when
# demo_robot.urdf.xacro became a thin wrapper over the upstream assembly:
#
#     laser_frame       -> rplidar_link
#     camera_link       -> oakd_rgb_camera_frame  (+ _optical_frame)
#     caster_wheel_link -> front_caster_link
#     left_wheel_link   -> left_wheel
#
# Nav2 uses base_link as robot_base_frame (see nav2_params.yaml): upstream's
# DiffDrive hardcodes odom -> base_link, and base_footprint_joint is an identity
# transform, so the two frames coincide.
#
# Sensor frames must match the gz_frame_id the Gazebo <sensor> blocks declare,
# otherwise scans and images arrive stamped with a frame_id absent from the TF
# tree and Nav2 silently discards them.
REQUIRED_LINKS = {
    'base_footprint',
    'base_link',
    'rplidar_link',
    'oakd_rgb_camera_frame',
    'imu_link',
    'left_wheel',
    'right_wheel',
    'front_caster_link',
}

# The DiffDrive plugin references these joint names verbatim. They must stay
# continuous, or locomotion silently breaks in ML3.
REQUIRED_CONTINUOUS_JOINTS = {'left_wheel_joint', 'right_wheel_joint'}

# Upstream's tree is rooted at base_link, with base_footprint as its CHILD.
# This is inverted relative to the old hand-built model. Because
# base_footprint_joint is xyz="0 0 0" rpy="0 0 0", the two frames are
# numerically identical and Nav2 is pointed at base_link.
EXPECTED_ROOT_LINK = 'base_link'


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


def test_sensors_are_rigidly_attached_to_base_link(urdf_root: ET.Element) -> None:
    """
    Sensors must reach base_link through fixed joints only.

    The invariant is rigidity, not a direct parent. The lidar and camera mount on
    shell_link (the sensor tower) and the camera sits several hops out via
    oakd_camera_bracket -> oakd_link -> oakd_rgb_camera_frame. What must never
    appear is a movable joint between a sensor and the body, which would make the
    sensor pose depend on /joint_states and silently desynchronise scans from TF.
    """
    parents = {
        joint.find('child').get('link'):
            (joint.find('parent').get('link'), joint.get('type'))
        for joint in urdf_root.findall('joint')
    }
    for frame in ('rplidar_link', 'oakd_rgb_camera_optical_frame', 'imu_link'):
        link, hops = frame, 0
        while link != 'base_link':
            assert link in parents, f'{frame} does not reach base_link (stuck at {link})'
            parent, joint_type = parents[link]
            assert joint_type == 'fixed', (
                f'{frame} reaches base_link through a {joint_type} joint at '
                f'{link}; every hop must be fixed'
            )
            link, hops = parent, hops + 1
            assert hops < 10, f'{frame} parent chain looks cyclic'


def test_base_footprint_coincides_with_base_link(urdf_root: ET.Element) -> None:
    """
    base_footprint must be a fixed IDENTITY transform on base_link.

    Upstream inverts the usual convention: base_link is the root and
    base_footprint hangs off it, rather than the reverse. That is only acceptable
    because the joint is an exact identity (xyz 0 0 0, rpy 0 0 0), which makes
    odom -> base_link and odom -> base_footprint the same edge numerically.

    Nav2 is configured with robot_base_frame: base_link precisely because
    upstream's DiffDrive hardcodes that child frame. If a future upstream bump
    gives this joint a real offset, that equivalence breaks: Nav2 would then plan
    against a frame displaced from the odometry child, and the robot would track
    paths with a constant offset. Fail loudly here instead.
    """
    joint = next(
        j for j in urdf_root.findall('joint')
        if j.find('child').get('link') == 'base_footprint'
    )
    assert joint.find('parent').get('link') == 'base_link'
    assert joint.get('type') == 'fixed'

    origin = joint.find('origin')
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        assert xyz == pytest.approx([0.0, 0.0, 0.0], abs=1e-9), (
            f'base_footprint_joint must be an identity transform, got xyz={xyz}. '
            f'Nav2 uses base_link as robot_base_frame on the assumption these '
            f'frames coincide — see nav2_params.yaml'
        )
        assert rpy == pytest.approx([0.0, 0.0, 0.0], abs=1e-9), \
            f'base_footprint_joint must be an identity transform, got rpy={rpy}'


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


def test_parts_are_assembled_not_scattered(urdf_root: ET.Element) -> None:
    """
    Every link must sit within the physical envelope of the robot.

    Regression test for a real bug: the tower and sensor offsets were
    transplanted from the upstream TB4 description, where they are measured
    from shell_link, but re-parented onto base_link. Each part was then offset
    by the shell height twice and the robot rendered as a pile of disconnected
    pieces floating around a chassis.

    Nothing errored — the URDF was valid, TF was consistent and odometry was
    correct. The only symptom was visual, which is exactly the class of failure
    that survives every other check in this file.

    That specific bug is now structurally impossible: demo_robot.urdf.xacro
    includes the upstream assembly instead of re-deriving offsets. The test stays
    as a tripwire for the next person who re-adds hand-placed parts, and to catch
    an upstream bump that changes the robot's overall dimensions.
    """
    joints = {
        joint.find('child').get('link'):
            (joint.find('parent').get('link'),
             [float(v) for v in (joint.find('origin').get('xyz', '0 0 0').split()
                                 if joint.find('origin') is not None else ['0', '0', '0'])])
        for joint in urdf_root.findall('joint')
    }

    def absolute(link: str) -> list:
        pose, hops = [0.0, 0.0, 0.0], 0
        while link in joints and hops < 10:
            link, xyz = joints[link]
            pose = [a + b for a, b in zip(pose, xyz)]
            hops += 1
        return pose

    # Measuring gaps between link ORIGINS does not work: an origin is not where
    # the part is. The standoff mesh is centred on its origin and reaches
    # 10.5 cm either side, so origin-to-origin distance reads as a 0.27 m "gap"
    # between parts that physically touch.
    #
    # Overall height is the honest proxy, and it needs no mesh parsing. The
    # assembled robot stands 0.346 m; every scattered variant overshot it
    # because the doubled offsets pushed the tower up. A robot of this class is
    # never taller than half a metre.
    top = max(absolute(link)[2] for link in
              (element.get('name') for element in urdf_root.findall('link')))
    assert top <= 0.40, (
        f'highest link origin sits at z={top:.3f} m; this robot stands ~0.35 m. '
        f'Tower offsets are measured from shell_link — check they are not being '
        f'applied from base_link as well, which stacks them twice'
    )


def test_exactly_one_of_each_gz_system_plugin(urdf_root: ET.Element) -> None:
    """
    Each Gazebo system plugin must appear exactly once.

    Regression test for a trap hit while writing the wrapper. xacro does NOT
    override or merge <gazebo> blocks — it CONCATENATES them. So re-declaring
    upstream's DiffDrive here to change one field (child_frame_id) does not
    replace it: the expanded URDF ends up with two DiffDrive plugins bound to the
    same two wheel joints, both integrating odometry and both publishing TF.

    Gazebo loads both without complaint. There is no error message.
    """
    counts: dict[str, int] = {}
    for gz in urdf_root.findall('gazebo'):
        for plugin in gz.findall('plugin'):
            name = plugin.get('name')
            counts[name] = counts.get(name, 0) + 1

    duplicated = {n: c for n, c in counts.items() if c > 1}
    assert not duplicated, (
        f'duplicated Gazebo system plugins: {duplicated}. xacro concatenates '
        f'<gazebo> blocks rather than overriding them — to change an upstream '
        f'plugin field you must patch it upstream or reconcile on the consumer '
        f'side, not re-declare the plugin in demo_robot.urdf.xacro'
    )


def test_weld_script_removes_every_preserve_fixed_joint() -> None:
    """
    The weld script must leave zero <preserveFixedJoint> tags, and must be silent.

    Regression test for the actual cause of the long-running "robot with expanded
    parts" symptom. Upstream tags 21+ fixed joints with preserveFixedJoint, which
    stops Gazebo welding each joint's child into its parent's rigid body. The
    robot then spawns as 13 independent physics bodies that drift apart under
    gravity — the tower leans, the sensor plate and lidar float off the standoffs.

    It is NOT a geometry bug: RViz2 ignores <gazebo> blocks and always drew the
    robot correctly, which is why re-measuring mesh offsets and swapping the whole
    robot model both failed to fix it.

    The silence assertion is not cosmetic. launch's Command substitution aborts
    the entire launch if the command writes ANYTHING to stderr, so a progress
    message in the script takes the simulation down with "executed command showed
    stderr output".
    """
    share = get_package_share_directory('demo_description')
    script = share + '/scripts/weld_fixed_joints.py'
    model = share + '/urdf/demo_robot.urdf.xacro'

    result = subprocess.run(
        ['python3', script, model],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, f'weld script failed:\n{result.stderr}'
    assert result.stderr == '', (
        f'weld script must be silent on success — launch treats any stderr as '
        f'fatal and aborts. Got: {result.stderr!r}'
    )
    assert 'preserveFixedJoint' not in result.stdout, \
        'preserveFixedJoint survived; the robot will come apart in Gazebo'

    # The filter must not have eaten anything else from the <gazebo> blocks.
    welded = ET.fromstring(result.stdout)
    plugins = [
        p.get('name') for gz in welded.findall('gazebo') for p in gz.findall('plugin')
    ]
    assert plugins.count('gz::sim::systems::DiffDrive') == 1
    assert plugins.count('gz::sim::systems::JointStatePublisher') == 1

    sensors = [s.get('name') for gz in welded.findall('gazebo') for s in gz.findall('sensor')]
    assert set(sensors) == {'imu', 'rplidar', 'rgbd_camera'}, \
        f'weld script dropped a sensor: {sensors}'

    friction = sum(1 for gz in welded.findall('gazebo') if gz.find('mu1') is not None)
    assert friction == 3, f'weld script dropped wheel friction blocks: {friction}'


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
