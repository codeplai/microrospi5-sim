import os
from glob import glob

from setuptools import setup

package_name = 'yahboomcar_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='docente',
    maintainer_email='codeplaigamessac@gmail.com',
    description='Simulador cinematico 2D del robot Yahboom para RViz',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_robot = yahboomcar_sim.sim_node:main',
            'autopilot = yahboomcar_sim.autopilot_node:main',
            'gen_tracks = yahboomcar_sim.track_gen:main',
        ],
    },
)
