from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'demo_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Modelos spawnaveis (ros_gz_sim create -file). Hoje so as cameras
        # de cena do cockpit; ver launch/scene_cameras.launch.py.
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Maia',
    maintainer_email='engdiegomaia@users.noreply.github.com',
    description='Gazebo Harmonic world, robot spawn, ros_gz_bridge and teleop (x86 host only).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ML3.5 F3. Translates the topic contract's /demo/cmd_vel into the
            # gait controller's /control_input and walks the gait FSM up to
            # TROTTING. Lives here, next to Gazebo, because it is part of the
            # plant — nothing outside the sim container knows it exists.
            'twist_to_inputs = demo_simulation.twist_to_inputs:main',
            # Republica /clock a taxa fixa. Sem ele o clock de 1 kHz do passo
            # de fisica satura o Aquila AM69 no modo hil.
            'clock_throttle = demo_simulation.clock_throttle:main',
            # Câmera orbital das vistas de cena. Ela é quem sabe onde as
            # câmeras estão; o cockpit só publica passos relativos.
            'scene_view_controller = '
            'demo_simulation.scene_view_controller:main',
            # Fachada std_srvs para play/pause/reset. Existe porque o container
            # do cockpit não tem (nem deve ter) ros_gz_interfaces.
            'sim_control_relay = demo_simulation.sim_control_relay:main',
            'maze_escape_validator = '
            'demo_simulation.maze_escape_validator:main',
        ],
    },
)
