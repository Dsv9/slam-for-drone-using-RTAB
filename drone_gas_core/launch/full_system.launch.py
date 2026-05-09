from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_core = get_package_share_directory("drone_gas_core")
    pkg_sim_bridge = get_package_share_directory("drone_gas_sim_bridge")
    start_bridge = LaunchConfiguration("start_bridge")
    gazebo_twist_topic = LaunchConfiguration("gazebo_twist_topic")
    enable_exploration = LaunchConfiguration("enable_exploration")
    return LaunchDescription(
        [
            # Frame chain for simulation SLAM:
            #   odom -> base_link -> rgbd_camera
            # Gazebo camera message headers are currently using:
            #   simple_drone/base_link/rgbd_camera
            # We publish a static alias tf to that exact frame so RTAB-Map and RViz
            # can transform camera data consistently without changing topic names.
            #
            # RViz fixed frame should start as base_link or odom while map is not yet
            # being published/initialized. Using map too early causes filter drops.
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("gazebo_twist_topic", default_value="/cmd_vel"),
            # Exploration fights VO smoke tests — enable only for full demo.
            DeclareLaunchArgument("enable_exploration", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_sim_bridge, "launch", "spawn_simple_drone.launch.py"
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_core, "launch", "rtabmap_rgbd.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "rgb_topic": "/rgbd_camera/image",
                    "depth_topic": "/rgbd_camera/depth_image",
                    "camera_info_topic": "/rgbd_camera/camera_info",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(pkg_core, "rviz", "drone_gas_core.rviz"),
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="rgbd_camera_static_tf",
                output="screen",
                # Must match RGBD sensor pose in drone_gas_sim_bridge/models/simple_drone/model.sdf
                # Match model.sdf camera pose: x y z yaw pitch roll (see Gazebo sdf rpy).
                arguments=[
                    "0.30",
                    "0",
                    "0.055",
                    "0",
                    "-0.06",
                    "0",
                    "base_link",
                    "rgbd_camera",
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="rgbd_camera_gazebo_frame_alias_tf",
                output="screen",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "rgbd_camera",
                    "simple_drone/base_link/rgbd_camera",
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="drone_gas_core",
                executable="gas_sensor_sim_node",
                name="gas_sensor_sim_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_core, "config", "gas_sensor_sim.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="drone_gas_core",
                executable="chemical_mapper_node",
                name="chemical_mapper_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_core, "config", "chemical_mapper.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="drone_gas_core",
                executable="exploration_controller_node",
                name="exploration_controller_node",
                output="screen",
                condition=IfCondition(enable_exploration),
                parameters=[
                    os.path.join(pkg_core, "config", "exploration_controller.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="drone_gas_core",
                executable="cmd_vel_watchdog_node",
                name="cmd_vel_watchdog_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_core, "config", "cmd_vel_watchdog.yaml"),
                    {
                        # Allow slow open-loop motion before rgbd_odometry publishes valid odom
                        # (exploration + smoke tests). Set true for hardware safety if repurpose.
                        "require_odom": False,
                        "debug_print_cmd_vel": False,
                        "use_sim_time": True,
                    },
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
