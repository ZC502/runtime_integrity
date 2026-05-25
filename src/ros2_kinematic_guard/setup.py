from setuptools import setup
import os
from glob import glob

package_name = "ros2_kinematic_guard"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="your_email@example.com",
    description="NARH-based Kinematic Guard for ROS 2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "jitter_injector_node = ros2_kinematic_guard.jitter_injector_node:main",
            "kinematic_guard_node = ros2_kinematic_guard.kinematic_guard_node:main",
            "synthetic_odom_provider = ros2_kinematic_guard.synthetic_odom_provider:main",
            "mock_robot_simulator = ros2_kinematic_guard.mock_robot_simulator:main",
            "command_integrity_reporter_node = ros2_kinematic_guard.reporter_node:main",
            "execution_observer_node = ros2_kinematic_guard.execution_observer_node:main",
        ],
    },
)
