"""Slow /drone/cmd_vel motion for rgbd_odometry in Gazebo.

Use with full_system.launch.py and enable_exploration:=false so this is the only
publisher on /drone/cmd_vel.

Phases:
  1) Short warmup: slow forward, zero yaw (helps VO bootstrap).
  2) Keeps creeping forward while waiting until /rtabmap/odom_info reports enough
     inliers (or timeout).
  3) Optional yaw scan legs only after that gate (or timeout fallback).

Subscribe to RTAB‘s odom_info (not nav_msgs/Odometry) — it exposes visual inliers.
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from rtabmap_msgs.msg import OdomInfo
except ImportError:
    OdomInfo = None  # type: ignore[misc, assignment]


class _SmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("visual_odometry_smoke_motion")
        self.declare_parameter("linear_x", 0.035)
        self.declare_parameter("creep_x", 0.025)
        self.declare_parameter("yaw_z", 0.0)
        self.declare_parameter("warmup_sec", 5.0)
        self.declare_parameter("phase2_max_sec", 30.0)
        self.declare_parameter("rate_hz", 8.0)
        self.declare_parameter("scan_enabled", True)
        self.declare_parameter("scan_wz", 0.045)
        self.declare_parameter("scan_leg_sec", 4.0)
        self.declare_parameter("scan_min_inliers", 12)
        self.declare_parameter("scan_wait_timeout_sec", 22.0)
        self.declare_parameter("odom_info_topic", "/rtabmap/odom_info")

        self._inliers = -1
        self._last_odom_stamp = None

        if OdomInfo is not None:
            qos_odom = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                OdomInfo,
                self.get_parameter("odom_info_topic").value,
                self._odom_info_cb,
                qos_odom,
            )
        else:
            self.get_logger().warn(
                "rtabmap_msgs not installed: scan phase uses time fallback only "
                "(install ros-jazzy-rtabmap-msgs or distro equivalent)."
            )

    def _odom_info_cb(self, msg) -> None:
        self._inliers = int(msg.inliers)
        try:
            self._last_odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception:
            self._last_odom_stamp = time.time()


def main() -> None:
    rclpy.init()
    node = _SmokeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    lx = float(node.get_parameter("linear_x").value)
    creep = float(node.get_parameter("creep_x").value)
    wz_fwd = float(node.get_parameter("yaw_z").value)
    warmup = float(node.get_parameter("warmup_sec").value)
    p2_max = float(node.get_parameter("phase2_max_sec").value)
    hz = float(node.get_parameter("rate_hz").value)
    scan_on = bool(node.get_parameter("scan_enabled").value)
    wz_scan = float(node.get_parameter("scan_wz").value)
    leg = float(node.get_parameter("scan_leg_sec").value)
    need_inl = int(node.get_parameter("scan_min_inliers").value)
    wait_timeout = float(node.get_parameter("scan_wait_timeout_sec").value)

    dt = max(1.0 / hz, 0.05)
    pub = node.create_publisher(Twist, "/drone/cmd_vel", 10)

    def spin_spin() -> None:
        executor.spin_once(timeout_sec=0.02)

    def pub_cmd(vx: float, wz: float) -> None:
        m = Twist()
        m.linear.x = vx
        m.angular.z = wz
        pub.publish(m)

    node.get_logger().info(
        "VO smoke: warmup=%.1fs lx=%s then creep=%s until inliers≥%s or %.1fs; scan=%s"
        % (warmup, lx, creep, need_inl, wait_timeout, scan_on)
    )

    t0 = time.time()
    # Phase 1 — forward only
    while time.time() - t0 < warmup and rclpy.ok():
        pub_cmd(lx, wz_fwd)
        spin_spin()
        time.sleep(dt)

    # Phase 2 — keep forward slowly until quality or timeout
    t_gate = time.time()
    deadline = t_gate + wait_timeout
    phase_deadline = time.time() + p2_max
    while rclpy.ok() and time.time() < phase_deadline:
        pub_cmd(creep, 0.0)
        spin_spin()
        time.sleep(dt)
        if OdomInfo is not None:
            if node._inliers >= need_inl and time.time() > t_gate + 2.0:
                node.get_logger().info(
                    "odom_info gate: inliers=%s ≥ %s" % (node._inliers, need_inl)
                )
                break
            if time.time() > deadline:
                node.get_logger().warn(
                    "odom_info gate timed out (last inliers=%s); continuing to scan anyway"
                    % node._inliers
                )
                break

    if scan_on:
        ns = max(int(leg / dt), 1)
        node.get_logger().info(
            "Scan: legs %i steps (~%.2fs each), |wz|=%s" % (ns, ns * dt, wz_scan)
        )
        for label, sg in [("scan_left", 1), ("scan_right", -1)]:
            for _ in range(ns):
                pub_cmd(0.0, abs(wz_scan) * sg)
                spin_spin()
                time.sleep(dt)
            node.get_logger().info(label)

    pub.publish(Twist())
    node.get_logger().info("Done (zero cmd_vel).")

    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
