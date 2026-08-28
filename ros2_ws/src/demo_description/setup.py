from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'demo_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # weld_fixed_joints.py is invoked by demo_simulation's launch file via a
        # share-path lookup, so it must be installed here and not only exist in
        # the source tree. Without it Gazebo spawns the robot as 13 loose physics
        # bodies and it falls apart — see the note in demo_robot.urdf.xacro.
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Maia',
    maintainer_email='engdiegomaia@users.noreply.github.com',
    description='Parameterized differential-drive robot model, TF tree, and RViz dev config.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
