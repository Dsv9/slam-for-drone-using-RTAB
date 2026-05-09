"""Slow open-loop cmd_vel burst for rgbd_odometry (Gazebo sim).

Publishes repeatedly to /drone/cmd_vel while the stack is running full_system (watchdog
expects fresh commands at least every cmd_timeout_sec).

Parameters (ROS 2 -p overrides):
  linear_x (float, default 0.05): very slow forward for VO tracking (m/s)
  yaw_z (float, default 0): body yaw rate (rad/s) — keep 0 until VO locks
  duration_sec (float, default 8): motion phase length
  rate_hz (float, default 8): publish rate during motion

Examples:
  ros2 run drone_gas_core visual_odometry_smoke_motion
  ros2 run drone_gas_core visual_odometry_smoke_motion --ros-args -p linear_x:=0.04 -p duration_sec:=12.0

One-liner (no extra script):
  ros2 topic pub --rate 8 /drone/cmd_vel geometry_msgs/msg/Twist \\
    '{linear: {x: 0.05}, angular: {z: 0.0}}' --times 64
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("visual_odometry_smoke_motion_once")
    node.declare_parameter("linear_x", 0.05)
    node.declare_parameter("yaw_z", 0.0)
    node.declare_parameter("duration_sec", 8.0)
    node.declare_parameter("rate_hz", 8.0)

    lx = float(node.get_parameter("linear_x").value)
    wz = float(node.get_parameter("yaw_z").value)
    dur = float(node.get_parameter("duration_sec").value)
    hz = float(node.get_parameter("rate_hz").value)

    dt = max(1.0 / hz, 0.05)
    n = max(int(dur / dt), 1)

    pub = node.create_publisher(Twist, "/drone/cmd_vel", 10)
    msg = Twist()
    msg.linear.x = lx
    msg.angular.z = wz

    node.get_logger().info(
        "Smoke motion vx=%s wz=%s for ~%.2fs @ %.1f Hz (%i publishes)"
        % (lx, wz, dur, hz, n)
    )

    for _ in range(n):
        pub.publish(msg)
        time.sleep(dt)

    pub.publish(Twist())
    node.get_logger().info("Done (zero cmd published once).")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
