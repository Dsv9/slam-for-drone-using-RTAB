"""Minimal 2D obstacle avoidance using the central frontal depth ROI.

Reads /rgbd_camera/depth_image and publishes geometry_msgs/Twist on /drone/cmd_vel.

Intended demo: fixed-height “ground robot” behavior (linear.x + angular.z only).

Do not run this together with exploration_controller_node or visual_odometry_smoke_motion —
only one publisher should drive /drone/cmd_vel."""
from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def _decode_depth(msg: Image) -> Optional[np.ndarray]:
    """Return HxW float depth in metres, or None on unsupported encoding."""
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        return None

    elem = msg.encoding.upper()
    if elem in ("32FC1", "FLOAT32"):
        bpp = np.dtype(np.float32).itemsize
        row_el = msg.step // bpp if msg.step and msg.step >= w * bpp else w
        need = row_el * h
        flat = np.frombuffer(msg.data, dtype=np.float32)
        if flat.size < need:
            return None
        return flat[:need].reshape(h, row_el)[:, :w]
    if elem == "16UC1":
        row_el = msg.step // 2 if msg.step and msg.step >= w * 2 else w
        need = row_el * h
        flat = np.frombuffer(msg.data, dtype=np.uint16)
        if flat.size < need:
            return None
        return flat[:need].reshape(h, row_el)[:, :w].astype(np.float32) / 1000.0
    return None


class SimpleDepthAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_depth_avoidance")
        self.declare_parameter("depth_topic", "/rgbd_camera/depth_image")
        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("safe_distance_m", 0.7)
        self.declare_parameter("max_range_m", 8.0)
        self.declare_parameter("forward_speed_m_s", 0.04)
        self.declare_parameter("turn_speed_rad_s", 0.25)
        self.declare_parameter("roi_row_frac_start", 0.25)
        self.declare_parameter("roi_row_frac_end", 0.65)
        self.declare_parameter("roi_col_frac_start", 0.40)
        self.declare_parameter("roi_col_frac_end", 0.60)

        qos_depth = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._topic = str(self.get_parameter("depth_topic").value)
        self.create_subscription(Image, self._topic, self._depth_cb, qos_depth)

        cv_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._pub = self.create_publisher(Twist, cv_topic, 10)

        hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / hz, self._tick)

        self._last_depth: Optional[np.ndarray] = None
        self._last_stamp = None
        self._warn_no_depth = False

    def _depth_cb(self, msg: Image) -> None:
        d = _decode_depth(msg)
        if d is not None:
            self._last_depth = d

    def _tick(self) -> None:
        d = self._last_depth
        safe = float(self.get_parameter("safe_distance_m").value)
        vmax = float(self.get_parameter("max_range_m").value)
        v_fwd = float(self.get_parameter("forward_speed_m_s").value)
        w_turn = float(self.get_parameter("turn_speed_rad_s").value)
        rs, re = float(self.get_parameter("roi_row_frac_start").value), float(
            self.get_parameter("roi_row_frac_end").value
        )
        cs, ce = float(self.get_parameter("roi_col_frac_start").value), float(
            self.get_parameter("roi_col_frac_end").value
        )

        twist = Twist()
        if d is None:
            if not self._warn_no_depth:
                self._warn_no_depth = True
                self.get_logger().warn("No depth decoded yet (%s)." % self._topic)
            self._pub.publish(twist)
            return

        h, w = d.shape[:2]
        r0 = int(max(0.0, min(rs, re)) * h)
        r1 = int(max(0.0, max(rs, re)) * h)
        c0 = int(max(0.0, min(cs, ce)) * w)
        c1 = int(max(0.0, max(cs, ce)) * w)
        r1 = max(r1, r0 + 1)
        c1 = max(c1, c0 + 1)

        roi = d[r0:r1, c0:c1]
        valid = np.isfinite(roi) & (roi > 1e-3) & (roi < vmax)

        twist.linear.z = 0.0
        twist.linear.y = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0

        if not np.any(valid):
            # No trustworthy reading in front cone — creep turn in place slowly.
            twist.angular.z = w_turn * 0.6
            self._pub.publish(twist)
            return

        dm = roi[valid].min()
        if dm > safe:
            twist.linear.x = v_fwd
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = w_turn
        self._pub.publish(twist)


def main() -> None:
    rclpy.init()
    node = SimpleDepthAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
