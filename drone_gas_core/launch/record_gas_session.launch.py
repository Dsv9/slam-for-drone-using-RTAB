from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_name = LaunchConfiguration("bag_name")
    return LaunchDescription([
        DeclareLaunchArgument("bag_name", default_value="drone_gas_session"),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record", "-o", bag_name,
                "/rtabmap/localization_pose", "/rtabmap/odom",
                "/gas/concentration", "/gas/chemical_map",
                "/drone/cmd_vel", "/drone/cmd_vel_safe",
            ],
            output="screen",
        ),
    ])
