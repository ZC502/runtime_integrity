from setuptools import setup
import os
from glob import glob

package_name = 'ros2_kinematic_guard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'jitter_injector_node = ros2_kinematic_guard.jitter_injector_node:main',
            'kinematic_guard_node = ros2_kinematic_guard.kinematic_guard_node:main',
            'synthetic_odom_provider = ros2_kinematic_guard.synthetic_odom_provider:main',
        ],
    },
)
