"""Contracts for the Go2 live occupancy-map pipeline."""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'ros2_ws/src/demo_bringup/launch/nav_quadruped.launch.py'
SLAM_LAUNCH = ROOT / 'ros2_ws/src/demo_navigation/launch/slam.launch.py'
SLAM_PARAMS = ROOT / 'ros2_ws/src/demo_navigation/config/slam_params.yaml'
GO2_PARAMS = ROOT / 'ros2_ws/src/demo_navigation/config/nav2_params_go2.yaml'
ALIGN8_PARAMS = ROOT / 'ros2_ws/src/demo_navigation/config/params-align8.yaml'
NAV_DOCKERFILE = ROOT / 'docker/nav/Dockerfile'
PACKAGE_XML = ROOT / 'ros2_ws/src/demo_navigation/package.xml'


def _launch_tree() -> ast.Module:
    return ast.parse(LAUNCH.read_text(encoding='utf-8'))


def _assignment(name: str) -> ast.Assign:
    return next(
        node for node in ast.walk(_launch_tree())
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name
                for target in node.targets)
    )


def test_nav_image_contains_both_mapping_runtime_packages() -> None:
    dockerfile = NAV_DOCKERFILE.read_text(encoding='utf-8')
    package = PACKAGE_XML.read_text(encoding='utf-8')
    assert 'ros-${ROS_DISTRO}-pointcloud-to-laserscan' in dockerfile
    assert 'ros-${ROS_DISTRO}-slam-toolbox' in dockerfile
    assert '<exec_depend>pointcloud_to_laserscan</exec_depend>' in package
    assert '<exec_depend>slam_toolbox</exec_depend>' in package


def test_slam_runtime_excludes_the_rviz_plugin_from_the_module() -> None:
    dockerfile = NAV_DOCKERFILE.read_text(encoding='utf-8')
    assert 'apt-get download ros-${ROS_DISTRO}-slam-toolbox' in dockerfile
    assert '/lib/libSlamToolboxPlugin.so' in dockerfile
    assert "grep -q 'not found'" in dockerfile
    install_block = dockerfile.split(
        'RUN apt-get update && apt-get install', 1)[1].split(
            'COPY --from=slam-toolbox-runtime', 1)[0]
    assert 'ros-${ROS_DISTRO}-slam-toolbox' not in install_block


def test_cloud_converter_uses_the_full_cloud_and_a_distinct_scan() -> None:
    source = ast.unparse(_assignment('cloud_to_scan'))
    assert "package='pointcloud_to_laserscan'" in source
    assert "('cloud_in', '/demo/scan_cloud')" in source
    assert "('scan', '/demo/scan_slam')" in source
    assert "'target_frame': 'base'" in source
    assert "'min_height': 0.12" in source


def test_slam_owns_map_to_odom_and_uses_the_go2_frame() -> None:
    tree = _launch_tree()
    map_identity_arg = next(
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and call.args and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'publish_map_identity'
    )
    keywords = {kw.arg: kw.value for kw in map_identity_arg.keywords if kw.arg}
    assert keywords['default_value'].value == 'false'

    slam = yaml.safe_load(SLAM_PARAMS.read_text(encoding='utf-8'))
    params = slam['slam_toolbox']['ros__parameters']
    assert params['base_frame'] == 'base'
    assert params['scan_topic'] == '/demo/scan_slam'

    slam_launch = SLAM_LAUNCH.read_text(encoding='utf-8')
    assert "LaunchConfiguration('scan_topic')" in slam_launch


def test_global_costmap_retains_slam_free_and_occupied_space() -> None:
    for path in (GO2_PARAMS, ALIGN8_PARAMS):
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
        params = document['global_costmap']['global_costmap']['ros__parameters']
        assert params['rolling_window'] is False
        assert params['plugins'][0] == 'static_layer'
        static = params['static_layer']
        assert static['map_subscribe_transient_local'] is True
        assert static['subscribe_to_updates'] is True
        # The live obstacle layer still clears current observations. Persistence
        # belongs to SLAM, which remembers both free and occupied cells.
        assert params['obstacle_layer']['cloud']['clearing'] is True
