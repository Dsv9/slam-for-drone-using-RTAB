"""Simple 2D depth avoidance with timed turn bursts and escape phases.

Reads /rgbd_camera/depth_image and publishes geometry_msgs/Twist on /drone/cmd_vel.

State machine:
  NORMAL — forward when clear; on blocked front, pick side from depth ROIs and enter AVOID_TURN.
  AVOID_TURN — in-place turn for avoid_turn_duration_sec only, then NORMAL (no endless spin).
  REVERSE_ESCAPE — short reverse, then ESCAPE_TURN.
  ESCAPE_TURN — timed opposite yaw, then NORMAL.

Stuck: in NORMAL only, when commanded motion matches “mostly turning / creeping” for stuck_timeout_sec.
"""
from __future__ import annotations

import time
from enum import IntEnum
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
    if rs >= 0.0 and re >= 0.0:
        rmin, rmax = rs, re
    if cs >= 0.0 and ce >= 0.0:
        cmin, cmax = cs, ce
    return rmin, rmax, cmin, cmax


def _frac_window(h: int, w: int, r0f: float, r1f: float, c0f: float, c1f: float) -> Tuple[int, int, int, int]:
    r0 = int(max(0.0, min(r0f, r1f)) * h)
    r1 = int(max(0.0, max(r0f, r1f)) * h)
    c0 = int(max(0.0, min(c0f, c1f)) * w)
    c1 = int(max(0.0, max(c0f, c1f)) * w)
    r1 = max(r1, r0 + 1)
    c1 = max(c1, c0 + 1)
    return r0, r1, c0, c1


def _min_depth(roi: np.ndarray, vmax: float) -> Tuple[float, int]:
    valid = np.isfinite(roi) & (roi > 1e-3) & (roi < vmax)
    n = int(np.count_nonzero(valid))
    if n == 0:
        return float("nan"), 0
    return float(roi[valid].min()), n


class _State(IntEnum):
    NORMAL = 0
    AVOID_TURN = 1
    REVERSE_ESCAPE = 2
    ESCAPE_TURN = 3


def _as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    return str(v).lower() in ("true", "1", "yes")


def _state_name(s: _State) -> str:
    if s == _State.NORMAL:
        return "NORMAL"
    if s == _State.AVOID_TURN:
        return "AVOID_TURN"
    if s == _State.REVERSE_ESCAPE:
        return "REVERSE_ESCAPE"
    return "ESCAPE_TURN"


class SimpleDepthAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_depth_avoidance")
        self.declare_parameter("depth_topic", "/rgbd_camera/depth_image")
        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("safe_distance_m", 0.35)
        self.declare_parameter("max_range_m", 3.0)
        self.declare_parameter("forward_speed_m_s", 0.025)
        self.declare_parameter("turn_speed_rad_s", 0.22)
        self.declare_parameter("roi_row_frac_min", 0.22)
        self.declare_parameter("roi_row_frac_max", 0.50)
        self.declare_parameter("roi_col_frac_min", 0.43)
        self.declare_parameter("roi_col_frac_max", 0.57)
        self.declare_parameter("roi_row_frac_start", -1.0)
        self.declare_parameter("roi_row_frac_end", -1.0)
        self.declare_parameter("roi_col_frac_start", -1.0)
        self.declare_parameter("roi_col_frac_end", -1.0)
        self.declare_parameter("debug_avoidance", False)
        self.declare_parameter("debug_avoidance_period_sec", 1.0)
        self.declare_parameter("no_reading_forward_m_s", 0.008)

        self.declare_parameter("avoid_turn_duration_sec", 1.2)

        self.declare_parameter("stuck_timeout_sec", 4.0)
        self.declare_parameter("reverse_speed_m_s", -0.015)
        self.declare_parameter("reverse_duration_sec", 1.0)
        self.declare_parameter("escape_turn_duration_sec", 1.5)
        self.declare_parameter("alternate_turn_direction", True)
        self.declare_parameter("side_roi_enabled", True)
        self.declare_parameter("side_roi_col_left_min", 0.08)
        self.declare_parameter("side_roi_col_left_max", 0.38)
        self.declare_parameter("side_roi_col_right_min", 0.62)
        self.declare_parameter("side_roi_col_right_max", 0.92)
        self.declare_parameter("side_depth_clear_margin_m", 0.10)
        self.declare_parameter("stuck_vx_threshold_m_s", 0.022)
        self.declare_parameter("stuck_wz_threshold_rad_s", 0.10)

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

        self._state = _State.NORMAL
        self._phase_t0 = 0.0
        self._stuck_accum = 0.0
        self._last_mono: Optional[float] = None
        self._last_stuck_wz_sign = 1.0
        self._alt_sign = 1.0
        self._avoid_turn_wz = 0.0
        self._escape_turn_wz = 0.0

    def _depth_cb(self, msg: Image) -> None:
        d = _decode_depth(msg)
        if d is not None:
            self._last_depth = d

    def _tick(self) -> None:
        now = time.monotonic()
        prev = self._last_mono
        dt = 0.0 if prev is None else max(0.0, now - prev)
        self._last_mono = now

        safe = float(self.get_parameter("safe_distance_m").value)
        vmax = float(self.get_parameter("max_range_m").value)
        v_fwd = float(self.get_parameter("forward_speed_m_s").value)
        w_turn = float(self.get_parameter("turn_speed_rad_s").value)
        rs, re, cs, ce = _row_col_fracs(self)
        dbg = _as_bool(self.get_parameter("debug_avoidance").value)
        dbg_period = max(float(self.get_parameter("debug_avoidance_period_sec").value), 0.25)
        v_no_read = max(float(self.get_parameter("no_reading_forward_m_s").value), 0.0)

        stuck_to = max(float(self.get_parameter("stuck_timeout_sec").value), 0.5)
        v_rev = float(self.get_parameter("reverse_speed_m_s").value)
        v_rev = min(v_rev, -1e-6)
        rev_dur = max(float(self.get_parameter("reverse_duration_sec").value), 0.1)
        esc_turn_dur = max(float(self.get_parameter("escape_turn_duration_sec").value), 0.1)
        avoid_turn_dur = max(float(self.get_parameter("avoid_turn_duration_sec").value), 0.05)
        side_on = _as_bool(self.get_parameter("side_roi_enabled").value)
        alt_on = _as_bool(self.get_parameter("alternate_turn_direction").value)
        slm = float(self.get_parameter("side_roi_col_left_min").value)
        slM = float(self.get_parameter("side_roi_col_left_max").value)
        srm = float(self.get_parameter("side_roi_col_right_min").value)
        srM = float(self.get_parameter("side_roi_col_right_max").value)
        side_margin = max(float(self.get_parameter("side_depth_clear_margin_m").value), 0.0)
        vx_stuck = float(self.get_parameter("stuck_vx_threshold_m_s").value)
        wz_stuck = float(self.get_parameter("stuck_wz_threshold_rad_s").value)

        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.y = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0

        action = "idle"
        d_front = float("nan")
        d_left = float("nan")
        d_right = float("nan")

        # --- REVERSE_ESCAPE ---
        if self._state == _State.REVERSE_ESCAPE:
            elapsed = now - self._phase_t0
            twist.linear.x = v_rev
            twist.angular.z = 0.0
            action = "reverse_escape"
            if elapsed >= rev_dur:
                self._state = _State.ESCAPE_TURN
                self._phase_t0 = now
                self._escape_turn_wz = -np.sign(self._last_stuck_wz_sign or 1.0) * abs(w_turn)
            self._stuck_accum = 0.0
            self._pub.publish(twist)
            self._log_debug(
                dbg, dbg_period, now, action, _state_name(self._state), d_front, d_left, d_right, twist
            )
            return

        # --- ESCAPE_TURN ---
        if self._state == _State.ESCAPE_TURN:
            elapsed = now - self._phase_t0
            twist.linear.x = 0.0
            twist.angular.z = float(self._escape_turn_wz)
            action = "escape_turn"
            if elapsed >= esc_turn_dur:
                self._state = _State.NORMAL
                if alt_on:
                    self._alt_sign *= -1.0
            self._stuck_accum = 0.0
            self._pub.publish(twist)
            self._log_debug(
                dbg, dbg_period, now, action, _state_name(self._state), d_front, d_left, d_right, twist
            )
            return

        # --- AVOID_TURN (timed burst only) ---
        if self._state == _State.AVOID_TURN:
            elapsed = now - self._phase_t0
            twist.linear.x = 0.0
            twist.angular.z = float(self._avoid_turn_wz)
            action = "avoid_turn"
            if elapsed >= avoid_turn_dur:
                self._state = _State.NORMAL
            self._stuck_accum = 0.0
            self._pub.publish(twist)
            self._log_debug(
                dbg, dbg_period, now, action, _state_name(self._state), d_front, d_left, d_right, twist
            )
            return

        # --- NORMAL ---
        d = self._last_depth
        if d is None:
            if not self._warn_no_depth:
                self._warn_no_depth = True
                self.get_logger().warn("No depth decoded yet (%s)." % self._topic)
            action = "no_frame"
            self._stuck_accum = 0.0
            self._pub.publish(twist)
            self._log_debug(
                dbg, dbg_period, now, action, _state_name(self._state), d_front, d_left, d_right, twist
            )
            return

        h, w = d.shape[:2]
        r0, r1, c0, c1 = _frac_window(h, w, rs, re, cs, ce)
        roi_f = d[r0:r1, c0:c1]
        d_front, n_front = _min_depth(roi_f, vmax)

        if side_on:
            lr0, lr1, lc0, lc1 = _frac_window(h, w, rs, re, slm, slM)
            rr0, rr1, rc0, rc1 = _frac_window(h, w, rs, re, srm, srM)
            d_left, _ = _min_depth(d[lr0:lr1, lc0:lc1], vmax)
            d_right, _ = _min_depth(d[rr0:rr1, rc0:rc1], vmax)

        entering_avoid = False
        if not np.isfinite(d_front) or n_front == 0:
            twist.linear.x = v_no_read
            base_wz = w_turn * 0.55
            twist.angular.z = self._biased_turn_wz(
                base_wz, d_left, d_right, side_on, side_margin, alt_on
            )
            action = "no_valid_depth"
        elif d_front > safe:
            twist.linear.x = v_fwd
            twist.angular.z = 0.0
            action = "forward"
        else:
            entering_avoid = True
            self._state = _State.AVOID_TURN
            self._phase_t0 = now
            self._avoid_turn_wz = self._biased_turn_wz(
                w_turn, d_left, d_right, side_on, side_margin, alt_on
            )
            twist.linear.x = 0.0
            twist.angular.z = float(self._avoid_turn_wz)
            action = "avoid_turn_start"

        wz_abs = abs(twist.angular.z)
        vx_abs = abs(twist.linear.x)
        is_stuck = wz_abs >= wz_stuck and vx_abs <= vx_stuck
        if not entering_avoid and is_stuck:
            self._stuck_accum += dt
        else:
            self._stuck_accum = 0.0

        if self._stuck_accum >= stuck_to:
            self._last_stuck_wz_sign = float(np.sign(twist.angular.z) or 1.0)
            self._state = _State.REVERSE_ESCAPE
            self._phase_t0 = now
            self._stuck_accum = 0.0
            twist.linear.x = v_rev
            twist.angular.z = 0.0
            action = "reverse_escape"

        self._pub.publish(twist)
        self._log_debug(
            dbg, dbg_period, now, action, _state_name(self._state), d_front, d_left, d_right, twist
        )

    def _biased_turn_wz(
        self,
        base_mag: float,
        d_left: float,
        d_right: float,
        side_on: bool,
        margin_m: float,
        alt_on: bool,
    ) -> float:
        """Positive wz = CCW (turn left). Clearer left -> +wz; clearer right -> -wz."""
        if not side_on or (not np.isfinite(d_left) and not np.isfinite(d_right)):
            return base_mag * self._alt_sign if alt_on else base_mag
        if not np.isfinite(d_left):
            return -abs(base_mag)
        if not np.isfinite(d_right):
            return abs(base_mag)
        if d_left > d_right + margin_m:
            return abs(base_mag)
        if d_right > d_left + margin_m:
            return -abs(base_mag)
        return base_mag * self._alt_sign if alt_on else base_mag

    def _log_debug(
        self,
        dbg: bool,
        dbg_period: float,
        now: float,
        action: str,
        state_name: str,
        d_front: float,
        d_left: float,
        d_right: float,
        twist: Twist,
    ) -> None:
        if not dbg:
            return
        if now - self._dbg_last_t < dbg_period:
            return
        self._dbg_last_t = now
        df = "%.3f" % d_front if np.isfinite(d_front) else "nan"
        dl = "%.3f" % d_left if np.isfinite(d_left) else "nan"
        dr = "%.3f" % d_right if np.isfinite(d_right) else "nan"
        self.get_logger().info(
            "avoidance: state=%s action=%s d_front=%s d_left=%s d_right=%s vx=%.4f wz=%.4f stuck_accum=%.2fs"
            % (
                state_name,
                action,
                df,
                dl,
                dr,
                twist.linear.x,
                twist.angular.z,
                self._stuck_accum,
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