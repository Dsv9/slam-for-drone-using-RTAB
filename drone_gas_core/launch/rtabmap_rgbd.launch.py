import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Passed to rgbd_odometry argv (always applied — stock rtabmap.launch.py ignores params_file).
# Matches=7 / inliers=0 often clears up once Vis/MinInliers is not the default 20 and
# Vis/InlierDistance tolerates noisy Gazebo depth/projection.
_DEMO_ODOM_ARGS = (
    "--Vis/MinInliers 10 "
    "--Vis/MaxFeatures 5000 "
    "--Kp/MaxFeatures 5000 "
    "--Vis/MinFeatures 8 "
    "--Vis/InlierDistance 0.22 "
    "--Vis/CorNNDR 0.78 "
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
                    "approx_sync_max_interval": "0.25",
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    # Stock launch does not consume params_file; keep path for reference tooling.
                    "odom_args": _DEMO_ODOM_ARGS,
                }.items(),
            ),
        ]
    )
