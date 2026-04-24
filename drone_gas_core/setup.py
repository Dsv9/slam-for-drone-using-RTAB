from setuptools import find_packages, setup
from glob import glob
import os

package_name = "drone_gas_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Drone gas detection core package.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gas_sensor_sim_node = drone_gas_core.gas_sensor_sim_node:main",
            "chemical_mapper_node = drone_gas_core.chemical_mapper_node:main",
            "exploration_controller_node = drone_gas_core.exploration_controller_node:main",
            "cmd_vel_watchdog_node = drone_gas_core.cmd_vel_watchdog_node:main",
        ],
    },
)
