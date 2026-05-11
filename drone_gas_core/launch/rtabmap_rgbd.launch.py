import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Passed to rgbd_odometry argv (stock rtabmap.launch.py ignores arbitrary params_file).
#
# IMPORTANT: RTAB‘s default Vis/EstimationType=1 uses 3D→2D (PnP); Vis/InlierDistance is
# only used for EstimationType=0 (3D→3D). For Gazebo RGB-D noise, switching to type 0 and
# raising the 3D correspondence distance avoids “matches>0 but 0 geometric inliers” traps.
_DEMO_ODOM_ARGS = (
    "--Vis/EstimationType 0 "
    "--Vis/MinInliers 8 "
    "--Vis/MaxFeatures 8000 "
    "--Kp/MaxFeatures 8000 "
    "--Vis/MinFeatures 6 "
    "--Vis/InlierDistance 0.52 "
    "--Vis/PnPReprojError 10 "
    "--Vis/CorNNDR 0.82 "
    "--Vis/MinDepth 0.25 "
    "--Vis/MaxDepth 8 "
    "--Kp/MinDepth 0.2 "
    "--Kp/MaxDepth 8 "
    "--GFTT/MinDistance 1 "
    "--Reg/Force3DoF true "
    "--Odom/ImageDecimation 1"
)


def generate_launch_description():
    pkg = get_package_share_directory("drone_gas_core")
    rtab = get_package_share_directory("rtabmap_launch")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rgb_topic", default_value="/rgbd_camera/image"),
            DeclareLaunchArgument("depth_topic", default_value="/rgbd_camera/depth_image"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/rgbd_camera/camera_info"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rtab, "launch", "rtabmap.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_id": "base_link",
                    "subscribe_rgb": "true",
                    "subscribe_depth": "true",
                    "approx_sync": "true",
                    "queue_size": "50",
                    "topic_queue_size": "50",
                    "sync_queue_size": "50",
                    # Looser sync reduces dropped pairs when Gazebo stamps wobble slightly.
                    "approx_sync_max_interval": "0.35",
                    "depth_scale": "1.0",
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    # Stock launch does not consume params_file; keep path for reference tooling.
                    "odom_args": _DEMO_ODOM_ARGS,
                }.items(),
            ),
        ]
    )
