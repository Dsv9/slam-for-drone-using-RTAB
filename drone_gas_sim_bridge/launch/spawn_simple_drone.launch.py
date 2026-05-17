import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("drone_gas_sim_bridge")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    model_path = os.path.join(pkg_share, "models", "simple_drone", "model.sdf")
    world_path = os.path.join(pkg_share, "worlds", "empty_world.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    # Open lane along +X; camera faces vo_calib_wall (high-contrast, no collision).
    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "default",
            "-name",
            "simple_drone",
            "-file",
            model_path,
            "-x",
            "-3.5",
            "-y",
            "-3.05",
            "-z",
            "0.88",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    bridge_config = os.path.join(pkg_share, "config", "gz_ros_bridge.yaml")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
    )

    return LaunchDescription([gazebo, spawn_drone, bridge])
