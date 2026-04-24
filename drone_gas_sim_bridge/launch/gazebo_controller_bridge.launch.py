from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("drone_gas_sim_bridge")
    return LaunchDescription(
        [
            Node(
                package="drone_gas_sim_bridge",
                executable="gazebo_controller_bridge_node",
                name="gazebo_controller_bridge_node",
                output="screen",
                parameters=[os.path.join(pkg, "config", "gazebo_controller_bridge.yaml")],
            )
        ]
    )
