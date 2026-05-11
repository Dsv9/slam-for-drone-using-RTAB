"""Minimal 2D obstacle avoidance using the central frontal depth ROI.

Reads /rgbd_camera/depth_image and publishes geometry_msgs/Twist on /drone/cmd_vel.

Intended demo: fixed-height “ground robot” behavior (linear.x + angular.z only).

Do not run this together with exploration_controller_node or visual_odometry_smoke_motion —
only one publisher should drive /drone/cmd_vel.

Why the robot can “just revolve”: the controller publishes angular.z when (a) the ROI has
no valid depth pixels (NaN/inf/out of range), or (b) the nearest valid depth in the ROI is
<= safe_distance_m (obstacle / wall / floor too close in that window). Tune safe_distance_m,
ROI fractions, and max_range_m from launch until forward clears the threshold.
"""
from __future__ import annotations

import time
from typing import Optional, Tuple

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


def _row_col_fracs(node: Node) -> Tuple[float, float, float, float]:
    """ROI as fractions of image height/width [0,1]. Legacy start/end override min/max if set."""
    rmin = float(node.get_parameter("roi_row_frac_min").value)
    rmax = float(node.get_parameter("roi_row_frac_max").value)
    cmin = float(node.get_parameter("roi_col_frac_min").value)
    cmax = float(node.get_parameter("roi_col_frac_max").value)
    rs = float(node.get_parameter("roi_row_frac_start").value)
    re = float(node.get_parameter("roi_row_frac_end").value)
    cs = float(node.get_parameter("roi_col_frac_start").value)
    ce = float(node.get_parameter("roi_col_frac_end").value)
    # Legacy: if start/end differ from sentinels -1, use them (launch can omit min/max).
    if rs >= 0.0 and re >= 0.0:
        rmin, rmax = rs, re
    if cs >= 0.0 and ce >= 0.0:
        cmin, cmax = cs, ce
    return rmin, rmax, cmin, cmax


class SimpleDepthAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_depth_avoidance")
        self.declare_parameter("depth_topic", "/rgbd_camera/depth_image")
        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("safe_distance_m", 0.45)
        self.declare_parameter("max_range_m", 8.0)
        self.declare_parameter("forward_speed_m_s", 0.04)
        self.declare_parameter("turn_speed_rad_s", 0.25)
        # Preferred names (launch): frontal band in image fractions.
        self.declare_parameter("roi_row_frac_min", 0.22)
        self.declare_parameter("roi_row_frac_max", 0.50)
        self.declare_parameter("roi_col_frac_min", 0.43)
        self.declare_parameter("roi_col_frac_max", 0.57)
        # Legacy aliases (set to -1 to use min/max only).
        self.declare_parameter("roi_row_frac_start", -1.0)
        self.declare_parameter("roi_row_frac_end", -1.0)
        self.declare_parameter("roi_col_frac_start", -1.0)
        self.declare_parameter("roi_col_frac_end", -1.0)
        self.declare_parameter("debug_avoidance", False)
        self.declare_parameter("debug_avoidance_period_sec", 1.0)
        # When ROI has no valid depth, creep forward this much (m/s) while turning slowly.
        self.declare_parameter("no_reading_forward_m_s", 0.015)

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
        self._warn_no_depth = False
        self._dbg_last_t = 0.0

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
        rs, re, cs, ce = _row_col_fracs(self)
        dbg_raw = self.get_parameter("debug_avoidance").value
        dbg = (
            dbg_raw
            if isinstance(dbg_raw, bool)
            else str(dbg_raw).lower() in ("true", "1", "yes")
        )
        dbg_period = max(float(self.get_parameter("debug_avoidance_period_sec").value), 0.25)
        v_no_read = max(float(self.get_parameter("no_reading_forward_m_s").value), 0.0)

        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.y = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0

        action = "idle"
        dm = float("nan")
        n_valid = 0

        if d is None:
            if not self._warn_no_depth:
                self._warn_no_depth = True
                self.get_logger().warn("No depth decoded yet (%s)." % self._topic)
            self._pub.publish(twist)
            action = "no_frame"
        else:
            h, w = d.shape[:2]
            r0 = int(max(0.0, min(rs, re)) * h)
            r1 = int(max(0.0, max(rs, re)) * h)
            c0 = int(max(0.0, min(cs, ce)) * w)
            c1 = int(max(0.0, max(cs, ce)) * w)
            r1 = max(r1, r0 + 1)
            c1 = max(c1, c0 + 1)

            roi = d[r0:r1, c0:c1]
            valid = np.isfinite(roi) & (roi > 1e-3) & (roi < vmax)
            n_valid = int(np.count_nonzero(valid))

            if not np.any(valid):
                twist.linear.x = v_no_read
                twist.angular.z = w_turn * 0.55
                action = "no_valid_depth"
            else:
                dm = float(roi[valid].min())
                if dm > safe:
                    twist.linear.x = v_fwd
                    twist.angular.z = 0.0
                    action = "forward"
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = w_turn
                    action = "turn_obstacle"

            self._pub.publish(twist)

        if dbg:
            now = time.monotonic()
            if now - self._dbg_last_t >= dbg_period:
                self._dbg_last_t = now
                self.get_logger().info(
                    "avoidance: action=%s roi_valid_px=%i d_min_m=%s safe_m=%s twist vx=%.4f wz=%.4f"
                    % (
                        action,
                        n_valid,
                        "%.3f" % dm if np.isfinite(dm) else "nan",
                        safe,
                        twist.linear.x,
                        twist.angular.z,
                    )
                )


def main() -> None:
    rclpy.init()
    node = SimpleDepthAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
