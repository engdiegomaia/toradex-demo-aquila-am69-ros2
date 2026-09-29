from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'demo_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # demo_view.rviz must be installed: learn.launch.py and viz.launch.py
        # resolve it through the share path. Without it RViz starts with no
        # config and shows an empty view.
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Maia',
    maintainer_email='engdiegomaia@users.noreply.github.com',
    description='Top-level launch composition, one file per container role.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Ordering gate for nav and perception. Replaces the wall-clock
            # timers that learn.launch.py used inside a single process tree —
            # see the module docstring for why they do not survive a container
            # boundary.
            'wait_for_clock = demo_bringup.wait_for_clock:main',
            'wait_for_tf = demo_bringup.wait_for_tf:main',
            # Exhibition loop. Stands in for Nav2 as the producer of
            # /demo/cmd_vel, so it lives with the commanders and not in
            # demo_simulation, which is part of the plant.
            'demo_routine = demo_bringup.demo_routine:main',
            # Closes the top of the TF tree. See the module header for the
            # two publishers that cannot coexist with it.
            'odom_tf = demo_bringup.odom_tf:main',
            # Patrols under Nav2. Sends GOALS, not velocities -- that is why
            # it steers around obstacles and demo_routine does not. The two
            # cannot run together; see the module header.
            'patrol_commander = demo_bringup.patrol_commander:main',
            # Unit boundary between Nav2 (SI) and the /demo/cmd_vel contract
            # (stick). See the module header: without it the robot moves at
            # 40% of what was requested and nothing flags it.
            'cmd_vel_si_to_stick = demo_bringup.cmd_vel_si_to_stick:main',
            # Operational telemetry from the target for the cockpit: commanded
            # axes, CPU, memory and temperature without depending on raw
            # /rosout.
            'target_monitor = demo_bringup.target_monitor:main',
        ],
    },
)
