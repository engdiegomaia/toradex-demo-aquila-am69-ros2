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
    maintainer_email='diego.maia@toradex.com',
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
        ],
    },
)
