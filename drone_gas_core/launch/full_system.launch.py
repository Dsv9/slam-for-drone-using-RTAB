from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_core = get_package_share_directory("drone_gas_core")
    pkg_sim_bridge = get_package_share_directory("drone_gas_sim_bridge")
    start_bridge = LaunchConfiguration("start_bridge")
    gazebo_twist_topic = LaunchConfiguration("gazebo_twist_topic")
    enable_exploration = LaunchConfiguration("enable_exploration")
    enable_avoidance = LaunchConfiguration("enable_avoidance")
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
            # Optional depth-based creep + turn-away (see simple_depth_avoidance_node.py).
            # When true: exploration_controller is OFF unless you bypass this expression.
            DeclareLaunchArgument("enable_avoidance", default_value="false"),
            # --- simple_depth_avoidance_node tuning (only used when enable_avoidance:=true) ---
            DeclareLaunchArgument("safe_distance_m", default_value="0.45"),
            DeclareLaunchArgument("forward_speed_m_s", default_value="0.04"),
            DeclareLaunchArgument("turn_speed_rad_s", default_value="0.25"),
            DeclareLaunchArgument("max_range_m", default_value="8.0"),
            DeclareLaunchArgument("avoidance_publish_hz", default_value="10.0"),
            DeclareLaunchArgument("roi_row_frac_min", default_value="0.22"),
            DeclareLaunchArgument("roi_row_frac_max", default_value="0.50"),
            DeclareLaunchArgument("roi_col_frac_min", default_value="0.43"),
            DeclareLaunchArgument("roi_col_frac_max", default_value="0.57"),
            DeclareLaunchArgument("no_reading_forward_m_s", default_value="0.015"),
            DeclareLaunchArgument("debug_avoidance", default_value="false"),
            DeclareLaunchArgument("debug_avoidance_period_sec", default_value="1.0"),
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
                    "-0.05",
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
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("enable_exploration"),
                            "' == 'true' and '",
                            LaunchConfiguration("enable_avoidance"),
                            "' != 'true'",
                        ]
                    )
                ),
                parameters=[
                    os.path.join(pkg_core, "config", "exploration_controller.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="drone_gas_core",
                executable="simple_depth_avoidance_node",
                name="simple_depth_avoidance_node",
                output="screen",
                condition=IfCondition(enable_avoidance),
                parameters=[
                    {
                        "depth_topic": "/rgbd_camera/depth_image",
                        "cmd_vel_topic": "/drone/cmd_vel",
                        "safe_distance_m": LaunchConfiguration("safe_distance_m"),
                        "forward_speed_m_s": LaunchConfiguration("forward_speed_m_s"),
                        "turn_speed_rad_s": LaunchConfiguration("turn_speed_rad_s"),
                        "max_range_m": LaunchConfiguration("max_range_m"),
                        "publish_hz": LaunchConfiguration("avoidance_publish_hz"),
                        "roi_row_frac_min": LaunchConfiguration("roi_row_frac_min"),
                        "roi_row_frac_max": LaunchConfiguration("roi_row_frac_max"),
                        "roi_col_frac_min": LaunchConfiguration("roi_col_frac_min"),
                        "roi_col_frac_max": LaunchConfiguration("roi_col_frac_max"),
                        "no_reading_forward_m_s": LaunchConfiguration(
                            "no_reading_forward_m_s"
                        ),
                        "debug_avoidance": LaunchConfiguration("debug_avoidance"),
                        "debug_avoidance_period_sec": LaunchConfiguration(
                            "debug_avoidance_period_sec"
                        ),
                        "use_sim_time": True,
                    }
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
