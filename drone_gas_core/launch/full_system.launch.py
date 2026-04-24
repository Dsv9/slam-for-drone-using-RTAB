from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("drone_gas_core")
    start_bridge = LaunchConfiguration("start_bridge")
    gazebo_twist_topic = LaunchConfiguration("gazebo_twist_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("gazebo_twist_topic", default_value="/cmd_vel"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "rtabmap_rgbd.launch.py")
                )
            ),
            Node(
                package="drone_gas_core",
                executable="gas_sensor_sim_node",
                name="gas_sensor_sim_node",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "gas_sensor_sim.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="drone_gas_core",
                executable="chemical_mapper_node",
                name="chemical_mapper_node",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "chemical_mapper.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="drone_gas_core",
                executable="exploration_controller_node",
                name="exploration_controller_node",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "exploration_controller.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="drone_gas_core",
                executable="cmd_vel_watchdog_node",
                name="cmd_vel_watchdog_node",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "cmd_vel_watchdog.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="drone_gas_sim_bridge",
                executable="gazebo_controller_bridge_node",
                name="gazebo_controller_bridge_node",
                output="screen",
                condition=IfCondition(start_bridge),
                parameters=[
                    {
                        "input_cmd_vel_topic": "/drone/cmd_vel_safe",
                        "command_mode": "velocity",
                        "gazebo_twist_topic": gazebo_twist_topic,
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
