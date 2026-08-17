"""Static checks for the Go2 side of the public ROS topic contract."""

from pathlib import Path

import yaml


EXPECTED_GZ_OUTPUTS = {
    '/demo/odom': 'nav_msgs/msg/Odometry',
    '/demo/scan': 'sensor_msgs/msg/LaserScan',
    '/demo/camera/image_raw': 'sensor_msgs/msg/Image',
    '/demo/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/demo/imu': 'sensor_msgs/msg/Imu',
}


def _bridge_entries():
    config = Path(__file__).parents[1] / 'config' / 'bridge_quadruped.yaml'
    return yaml.safe_load(config.read_text(encoding='utf-8'))


def test_gazebo_outputs_keep_public_names_and_types() -> None:
    """The quadruped swap must be invisible to ROS-side consumers."""
    actual = {
        item['ros_topic_name']: item['ros_type_name']
        for item in _bridge_entries()
    }

    for topic, message_type in EXPECTED_GZ_OUTPUTS.items():
        assert actual.get(topic) == message_type


def test_cmd_vel_is_not_forwarded_to_a_gazebo_drive_plugin() -> None:
    """Go2 cmd_vel stays ROS-native and is owned by twist_to_inputs."""
    bridged_topics = {item['ros_topic_name'] for item in _bridge_entries()}

    assert '/demo/cmd_vel' not in bridged_topics
