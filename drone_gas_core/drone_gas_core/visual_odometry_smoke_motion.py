"""Slow cmd_vel profile for rgbd_odometry (Gazebo). Publishes /drone/cmd_vel.

Keep exploration_controller disabled (full_system enable_exploration:=false) so this is
the only publisher on /drone/cmd_vel.

Parameters (ROS 2):
  linear_x (default 0.04): forward m/s during forward phase
  yaw_z (default 0): yaw rate during forward phase (keep ~0 until VO stable)
  duration_sec (default 10): forward phase length
  rate_hz (default 6): publish rate
  scan_enabled (default false): after forward, add slow left-then-right yaw “scan”
  scan_wz (default 0.05): magnitude of scan yaw (rad/s)
  scan_leg_sec (default 3.0): duration per scan direction

Example:
  ros2 run drone_gas_core visual_odometry_smoke_motion --ros-args \\
    -p linear_x:=0.04 -p scan_enabled:=true
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("visual_odometry_smoke_motion_once")
    node.declare_parameter("linear_x", 0.04)
    node.declare_parameter("yaw_z", 0.0)
    node.declare_parameter("duration_sec", 10.0)
    node.declare_parameter("rate_hz", 6.0)
    node.declare_parameter("scan_enabled", False)
    node.declare_parameter("scan_wz", 0.05)
    node.declare_parameter("scan_leg_sec", 3.0)

    lx = float(node.get_parameter("linear_x").value)
    wz_fwd = float(node.get_parameter("yaw_z").value)
    dur = float(node.get_parameter("duration_sec").value)
    hz = float(node.get_parameter("rate_hz").value)
    scan_on = bool(node.get_parameter("scan_enabled").value)
    wz_scan = float(node.get_parameter("scan_wz").value)
    leg = float(node.get_parameter("scan_leg_sec").value)

    dt = max(1.0 / hz, 0.08)
    n = max(int(dur / dt), 1)

    pub = node.create_publisher(Twist, "/drone/cmd_vel", 10)

    def burst(vx: float, wz: float, steps: int, label: str) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        node.get_logger().info("%s: vx=%s wz=%s for %i steps (~%.2fs)" % (label, vx, wz, steps, steps * dt))
        for _ in range(steps):
            pub.publish(msg)
            time.sleep(dt)

    node.get_logger().info(
        "Smoke: forward vx=%s wz=%s ~%.2fs @ %.1f Hz; scan=%s"
        % (lx, wz_fwd, dur, hz, scan_on)
    )

    burst(lx, wz_fwd, n, "forward")

    if scan_on:
        ns = max(int(leg / dt), 1)
        burst(0.0, abs(wz_scan), ns, "scan_left")
        burst(0.0, -abs(wz_scan), ns, "scan_right")

    pub.publish(Twist())
    node.get_logger().info("Done (sent zero cmd once).")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
