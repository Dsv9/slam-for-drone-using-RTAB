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

    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-world", "default", "-name", "simple_drone", "-file", model_path],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    return LaunchDescription([gazebo, spawn_drone, bridge])
