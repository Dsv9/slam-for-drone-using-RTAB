from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


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
                    # queue_size only sets deprecated sync fallback; upstream defaults
                    # topic_queue_size=10 unless set explicitly — that matches the "(current=10)" warning.
                    "queue_size": "40",
                    "topic_queue_size": "40",
                    "sync_queue_size": "40",
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "params_file": os.path.join(pkg, "config", "rtabmap_params.yaml"),
                }.items(),
            ),
        ]
    )
