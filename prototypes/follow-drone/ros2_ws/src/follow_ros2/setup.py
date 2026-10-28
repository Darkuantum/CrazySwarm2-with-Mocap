import os
from glob import glob

from setuptools import find_packages, setup

package_name = "follow_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="follow_drone",
    maintainer_email="hwehuang.stengg@gmail.com",
    description=(
        "Thin ROS 2 adapter nodes around follow_core: follow_node, "
        "detector_bridge, uwb_node, mapper_bridge. Foxy- and Humble-compatible."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follow_node = follow_ros2.follow_node:main",
            "detector_bridge = follow_ros2.detector_bridge:main",
            "uwb_node = follow_ros2.uwb_node:main",
            "mapper_bridge = follow_ros2.mapper_bridge:main",
        ],
    },
)
