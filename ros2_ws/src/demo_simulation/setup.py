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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Maia',
    maintainer_email='diego.maia@toradex.com',
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
        ],
    },
)
