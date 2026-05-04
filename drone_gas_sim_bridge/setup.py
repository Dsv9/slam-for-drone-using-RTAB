from glob import glob
import os
from setuptools import find_packages, setup

package_name = "drone_gas_sim_bridge"
model_files = [f for f in glob("models/**/*", recursive=True) if os.path.isfile(f)]
world_files = [f for f in glob("worlds/**/*", recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models", "simple_drone"), model_files),
        (os.path.join("share", package_name, "worlds"), world_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Gazebo bridge package for drone simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gazebo_controller_bridge_node = drone_gas_sim_bridge.gazebo_controller_bridge_node:main",
        ],
    },
)
