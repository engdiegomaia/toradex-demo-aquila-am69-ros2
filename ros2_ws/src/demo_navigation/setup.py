from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'demo_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Vendored Nav2 launch files. Note the *_launch.py pattern: upstream
        # names them navigation_launch.py, not navigation.launch.py, so the
        # glob above does NOT match them and they would silently fail to
        # install — bringup_launch.py would then be missing at runtime with a
        # "file not found" that names a path nobody edited.
        # The README ships too, so provenance travels with the copies.
        (os.path.join('share', package_name, 'launch', 'nav2_vendored'),
            glob('launch/nav2_vendored/*_launch.py')
            + glob('launch/nav2_vendored/README.md')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # The behavior tree has to be INSTALLED, not just versioned: the
        # bt_navigator receives an absolute path from share/, and if the file
        # isn't there, it fails to load the tree and no goal is accepted.
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
        # Both halves of the map must ship: the .yaml references the .pgm.
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Maia',
    maintainer_email='engdiegomaia@users.noreply.github.com',
    description='Nav2 parameters, static map and navigation launch (arch-neutral).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # std_srvs facade over the Nav2 lifecycle: the cockpit's "reset
            # goal". Lives here, and not in demo_bringup, because it runs in
            # the same container as Nav2 and it is its stack that restarts.
            'nav_control_relay = demo_navigation.nav_control_relay:main',
            'maze_explorer = demo_navigation.maze_explorer:main',
        ],
    },
)
