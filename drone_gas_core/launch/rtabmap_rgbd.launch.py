import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

# rgbd_odometry argv (stock rtabmap.launch.py applies these via odom_args).
# EstimationType=0 => 3D-3D (Vis/InlierDistance matters). Type=1 is PnP and often fails in Gazebo.
_DEMO_ODOM_ARGS = (
    "--Vis/EstimationType 0 "
    "--Vis/MinInliers 5 "
    "--Vis/MaxFeatures 1000 "
    "--Kp/MaxFeatures 1000 "
    "--Vis/MinFeatures 5 "
    "--Vis/InlierDistance 0.55 "
    "--Vis/PnPReprojError 10 "
    "--Vis/CorNNDR 0.80 "
    "--Vis/MinDepth 0.20 "
    "--Vis/MaxDepth 8 "
    "--Kp/MinDepth 0.20 "
    "--Kp/MaxDepth 8 "
    "--GFTT/MinDistance 3 "
    "--Reg/Force3DoF true "
    "--Odom/ImageDecimation 1 "
    "--Odom/ResetCountdown 1"
)

# rtabmap mapping node (separate from odom tuning above).
_DEMO_RTABMAP_ARGS = (
    "--RGBD/LinearUpdate 0.02 "
    "--RGBD/AngularUpdate 0.01 "
    "--Grid/FromDepth true "
    "--Grid/3D false"
)


def generate_launch_description():
    pkg = get_package_share_directory("drone_gas_core")
    rtab = get_package_share_directory("rtabmap_launch")
    debug_odom = LaunchConfiguration("debug_odom")
    odom_log_level = PythonExpression(
        ["'debug' if '", debug_odom, "' == 'true' else 'info'"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rgb_topic", default_value="/rgbd_camera/image"),
            DeclareLaunchArgument("depth_topic", default_value="/rgbd_camera/depth_image"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/rgbd_camera/camera_info"
            ),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("odom_frame_id", default_value="odom"),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            # Relative to namespace rtabmap -> /rtabmap/odom (project convention).
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("vo_frame_id", default_value="odom"),
            DeclareLaunchArgument("namespace", default_value="rtabmap"),
            DeclareLaunchArgument("debug_odom", default_value="true"),
            DeclareLaunchArgument("enable_rtabmap_viz", default_value="true"),
            DeclareLaunchArgument(
                "rtabmap_params_yaml",
                default_value=os.path.join(pkg, "config", "rtabmap_params.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rtab, "launch", "rtabmap.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                    "map_frame_id": LaunchConfiguration("map_frame_id"),
                    "namespace": LaunchConfiguration("namespace"),
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "vo_frame_id": LaunchConfiguration("vo_frame_id"),
                    "visual_odometry": "true",
                    "icp_odometry": "false",
                    "depth": "true",
                    "subscribe_rgb": "true",
                    "subscribe_rgbd": "false",
                    "subscribe_scan": "false",
                    "rgbd_sync": "false",
                    "approx_sync": "true",
                    "queue_size": "50",
                    "topic_queue_size": "50",
                    "sync_queue_size": "50",
                    "approx_sync_max_interval": "0.10",
                    "wait_for_transform": "0.30",
                    "depth_scale": "1.0",
                    "publish_tf_odom": "true",
                    "publish_tf_map": "true",
                    "qos": "2",
                    "qos_image": "2",
                    "qos_camera_info": "2",
                    "qos_odom": "2",
                    "odom_args": _DEMO_ODOM_ARGS,
                    "rtabmap_args": _DEMO_RTABMAP_ARGS,
                    "odom_log_level": odom_log_level,
                    "log_level": odom_log_level,
                    "rtabmap_viz": LaunchConfiguration("enable_rtabmap_viz"),
                    "rviz": "false",
                }.items(),
            ),
        ]
    )
