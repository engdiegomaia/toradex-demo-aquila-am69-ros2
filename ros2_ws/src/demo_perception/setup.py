from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'demo_perception'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Maia',
    maintainer_email='diego.maia@toradex.com',
    description='Deterministic synthetic-detection stub and costmap adapter (arch-neutral).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_stub = demo_perception.detection_stub:main',
            'detections_to_cloud = demo_perception.detections_to_cloud:main',
        ],
    },
)
