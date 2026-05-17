from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
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
    enable_rtabmap = LaunchConfiguration("enable_rtabmap")
    enable_rviz = LaunchConfiguration("enable_rviz")
    debug_odom = LaunchConfiguration("debug_odom")

    rtabmap_delayed = GroupAction(
        condition=IfCondition(enable_rtabmap),
        actions=[
            TimerAction(
                period=4.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_core, "launch", "rtabmap_rgbd.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": "true",
                            "rgb_topic": "/rgbd_camera/image",
                            "depth_topic": "/rgbd_camera/depth_image",
                            "camera_info_topic": "/rgbd_camera/camera_info",
                            "odom_topic": "/odom",
                            "debug_odom": debug_odom,
                            "enable_rtabmap_viz": "true",
                        }.items(),
                    )
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            # Frame chain: odom -> base_link -> rgbd_camera (+ Gazebo alias frame).
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("gazebo_twist_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("enable_exploration", default_value="false"),
            DeclareLaunchArgument("enable_avoidance", default_value="false"),
            DeclareLaunchArgument("enable_rtabmap", default_value="true"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("debug_odom", default_value="true"),
            # --- DEMO closed-loop avoidance (only when enable_avoidance:=true) ---
            DeclareLaunchArgument("safe_distance_m", default_value="0.30"),
            DeclareLaunchArgument("critical_distance_m", default_value="0.18"),
            DeclareLaunchArgument("clear_distance_m", default_value="0.45"),
            DeclareLaunchArgument("forward_speed_m_s", default_value="0.060"),
            DeclareLaunchArgument("slow_forward_speed_m_s", default_value="0.030"),
            DeclareLaunchArgument("turn_speed_rad_s", default_value="0.35"),
            DeclareLaunchArgument("search_turn_speed_rad_s", default_value="0.20"),
            DeclareLaunchArgument("reverse_speed_m_s", default_value="-0.040"),
            DeclareLaunchArgument("reverse_time_s", default_value="1.2"),
            DeclareLaunchArgument("recovery_turn_time_s", default_value="1.8"),
            DeclareLaunchArgument("stuck_timeout_s", default_value="2.5"),
            DeclareLaunchArgument("progress_epsilon_m", default_value="0.025"),
            DeclareLaunchArgument("max_range_m", default_value="3.0"),
            DeclareLaunchArgument("min_effective_linear_speed_m_s", default_value="0.025"),
            DeclareLaunchArgument("min_effective_turn_speed_rad_s", default_value="0.20"),
            DeclareLaunchArgument("avoidance_publish_hz", default_value="10.0"),
            DeclareLaunchArgument("debug_avoidance", default_value="true"),
            DeclareLaunchArgument("debug_avoidance_period_sec", default_value="1.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_sim_bridge, "launch", "spawn_simple_drone.launch.py"
                    )
                )
            ),
            rtabmap_delayed,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(enable_rviz),
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
                            enable_exploration,
                            "' == 'true' and '",
                            enable_avoidance,
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
                        "odom_topic": "/odom",
                        "cmd_vel_topic": "/drone/cmd_vel",
                        "safe_distance_m": LaunchConfiguration("safe_distance_m"),
                        "critical_distance_m": LaunchConfiguration("critical_distance_m"),
                        "clear_distance_m": LaunchConfiguration("clear_distance_m"),
                        "forward_speed_m_s": LaunchConfiguration("forward_speed_m_s"),
                        "slow_forward_speed_m_s": LaunchConfiguration(
                            "slow_forward_speed_m_s"
                        ),
                        "turn_speed_rad_s": LaunchConfiguration("turn_speed_rad_s"),
                        "search_turn_speed_rad_s": LaunchConfiguration(
                            "search_turn_speed_rad_s"
                        ),
                        "recovery_turn_time_s": LaunchConfiguration("recovery_turn_time_s"),
                        "max_range_m": LaunchConfiguration("max_range_m"),
                        "stuck_timeout_s": LaunchConfiguration("stuck_timeout_s"),
                        "progress_epsilon_m": LaunchConfiguration("progress_epsilon_m"),
                        "reverse_time_s": LaunchConfiguration("reverse_time_s"),
                        "reverse_speed_m_s": LaunchConfiguration("reverse_speed_m_s"),
                        "min_effective_linear_speed_m_s": LaunchConfiguration(
                            "min_effective_linear_speed_m_s"
                        ),
                        "min_effective_turn_speed_rad_s": LaunchConfiguration(
                            "min_effective_turn_speed_rad_s"
                        ),
                        "demo_avoidance_mode": True,
                        "publish_hz": LaunchConfiguration("avoidance_publish_hz"),
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
