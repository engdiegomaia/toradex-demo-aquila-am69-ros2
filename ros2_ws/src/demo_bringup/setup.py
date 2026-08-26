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
            # Exhibition loop. Stands in for Nav2 as the producer of
            # /demo/cmd_vel, so it lives with the commanders and not in
            # demo_simulation, which is part of the plant.
            'demo_routine = demo_bringup.demo_routine:main',
            # Fecha o topo da arvore TF. Ver o cabecalho do modulo para os
            # dois publicadores que nao podem coexistir com ele.
            'odom_tf = demo_bringup.odom_tf:main',
            # Patrulha sob Nav2. Manda METAS, nao velocidades -- e por isso que
            # ela desvia e demo_routine nao. Os dois nao podem rodar juntos;
            # ver o cabecalho do modulo.
            'patrol_commander = demo_bringup.patrol_commander:main',
            # Fronteira de unidades entre o Nav2 (SI) e o contrato
            # /demo/cmd_vel (manche). Ver o cabecalho do modulo: sem ele
            # o robo anda a 40% do pedido e nada acusa.
            'cmd_vel_si_to_stick = demo_bringup.cmd_vel_si_to_stick:main',
            # Telemetria operacional do target para o cockpit: eixos comandados,
            # CPU, memoria e temperatura sem depender de /rosout bruto.
            'target_monitor = demo_bringup.target_monitor:main',
        ],
    },
)
