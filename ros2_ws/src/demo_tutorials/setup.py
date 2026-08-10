from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'demo_tutorials'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Maia',
    maintainer_email='diego.maia@toradex.com',
    description='L1 learning code — heartbeat pub/sub, service example, launch composition.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_publisher = demo_tutorials.heartbeat_publisher:main',
            'heartbeat_subscriber = demo_tutorials.heartbeat_subscriber:main',
            'add_two_ints_server = demo_tutorials.add_two_ints_server:main',
        ],
    },
)
