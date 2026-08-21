"""Static checks for the Go2 side of the public ROS topic contract."""

from pathlib import Path
import re

import yaml


# Escala uniforme dos labirintos. Uniforme de proposito: escalar so x e y
# alargaria o corredor e deixaria a parede em 0.30 m, exatamente a altura do
# plano de varredura do lidar L1, que produz scan intermitente parecido com
# defeito de bridge.
MAZE_SCALE = '0.002 0.002 0.002'

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


def _maze_worlds():
    worlds = sorted((Path(__file__).parents[1] / 'worlds').glob('quadruped_maze*.sdf'))
    assert worlds, 'nenhum mundo de labirinto encontrado'
    return worlds


def test_maze_worlds_reference_the_mesh_directly() -> None:
    """
    An <include> of model://mazeN lays mazes 8-11 on their side.

    The upstream model.sdf declares roll 1.5708 for EVERY maze, including the
    ones whose STL is already Z-up. Its own worlds get away with it because
    they inline the link; an <include> here would stand the walls on the wrong
    axis. Referencing the mesh directly is what keeps rpy at 0 0 0.
    """
    for world in _maze_worlds():
        text = world.read_text(encoding='utf-8')
        body = re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)

        assert re.search(r'<uri>\s*model://maze\d+/meshes/maze\d+\.stl\s*</uri>', body), \
            f'{world.name}: malha nao referenciada direto por <uri>model://mazeN/meshes/'
        assert not re.search(r'<include>\s*<uri>\s*model://maze', body), \
            f'{world.name}: <include>model://mazeN deita o labirinto de lado'


def test_maze_worlds_keep_uniform_scale_and_upright_pose() -> None:
    """
    Scale is uniform and rpy is zero, or the lidar plane breaks.

    Both invariants are measured, not stylistic: uniform 0.002 puts the
    corridor at 1.20 m (against the 0.77 m the trunk needs) and the wall at
    0.60 m (clear of the L1 lidar seated at ~0.306 m).
    """
    for world in _maze_worlds():
        body = re.sub(
            r'<!--.*?-->', '', world.read_text(encoding='utf-8'), flags=re.DOTALL)

        scales = re.findall(r'<scale>\s*([^<]+?)\s*</scale>', body)
        assert scales, f'{world.name}: nenhuma <scale> declarada'
        for scale in scales:
            assert ' '.join(scale.split()) == MAZE_SCALE, \
                f'{world.name}: escala {scale!r} nao e a uniforme {MAZE_SCALE!r}'

        maze = re.search(
            r'<model name="labirinto">.*?<pose>\s*([^<]+?)\s*</pose>', body, re.DOTALL)
        assert maze, f'{world.name}: modelo "labirinto" sem <pose>'
        pose = maze.group(1).split()
        assert len(pose) == 6, f'{world.name}: <pose> com {len(pose)} campos, esperado 6'
        assert [float(v) for v in pose[3:]] == [0.0, 0.0, 0.0], \
            f'{world.name}: rpy {pose[3:]} nao e 0 0 0 (mazes 8-11 sao Z-up)'
