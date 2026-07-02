from setuptools import setup
import os
from glob import glob

package_name = "teleop_localization_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sab",
    maintainer_email="sab@example.de",
    description="Localization publisher + ROS 2 to WebSocket bridge (OvGU map demo).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "localization_node = teleop_localization_py.localization_node:main",
            "ws_bridge = teleop_localization_py.ws_bridge:main",
        ],
    },
)
