from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zach Gaier',
    maintainer_email='zgaier@wisc.edu',
    description='Gazebo Harmonic simulation of the holonomic FLOW base.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'generate_maze = robot_simulation.maze_generator:main',
        ],
    },
)
