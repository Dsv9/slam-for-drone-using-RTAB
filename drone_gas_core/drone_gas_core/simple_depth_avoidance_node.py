"""DEMO closed-loop depth avoidance — timer @10 Hz, /odom progress enforcement.

Publishes /drone/cmd_vel -> cmd_vel_watchdog -> /drone/cmd_vel_safe
  -> gazebo_controller_bridge_node -> /cmd_vel -> ros_gz_bridge -> Gazebo VelocityControl.
"""
from __future__ import annotations

import math
import time
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def _decode_depth(msg: Image) -> Optional[np.ndarray]:
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


def _frac_window(h: int, w: int, r0f: float, r1f: float, c0f: float, c1f: float) -> Tuple[int, int, int, int]:
    r0 = int(max(0.0, min(r0f, r1f)) * h)
    r1 = int(max(0.0, max(r0f, r1f)) * h)
    c0 = int(max(0.0, min(c0f, c1f)) * w)
    c1 = int(max(0.0, max(c0f, c1f)) * w)
    return r0, max(r1, r0 + 1), c0, max(c1, c0 + 1)


def _robust_distance(roi: np.ndarray, vmax: float, percentile: float) -> Tuple[float, int]:
    valid = np.isfinite(roi) & (roi > 1e-3) & (roi < vmax)
    n = int(np.count_nonzero(valid))
    if n == 0:
        return float("nan"), 0
    vals = np.clip(roi[valid], 1e-3, vmax)
    return float(np.percentile(vals, percentile)), n


class _State(IntEnum):
    FORWARD = 0
    SLOW_FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    REVERSE = 4
    RECOVERY_TURN = 5
    SEARCH_TURN = 6


def _as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    return str(v).lower() in ("true", "1", "yes")


class SimpleDepthAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_depth_avoidance")
        self.declare_parameter("demo_avoidance_mode", True)
        self.declare_parameter("depth_topic", "/rgbd_camera/depth_image")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("publish_hz", 10.0)

        self.declare_parameter("safe_distance_m", 0.30)
        self.declare_parameter("critical_distance_m", 0.18)
        self.declare_parameter("clear_distance_m", 0.45)
        self.declare_parameter("forward_speed_m_s", 0.060)
        self.declare_parameter("slow_forward_speed_m_s", 0.030)
        self.declare_parameter("turn_speed_rad_s", 0.35)
        self.declare_parameter("search_turn_speed_rad_s", 0.20)
        self.declare_parameter("reverse_speed_m_s", -0.040)
        self.declare_parameter("reverse_time_s", 1.2)
        self.declare_parameter("recovery_turn_time_s", 1.8)
        self.declare_parameter("stuck_timeout_s", 2.5)
        self.declare_parameter("progress_epsilon_m", 0.025)
        self.declare_parameter("max_range_m", 3.0)
        self.declare_parameter("min_effective_linear_speed_m_s", 0.025)
        self.declare_parameter("min_effective_turn_speed_rad_s", 0.20)
        self.declare_parameter("slow_forward_max_s", 2.0)
        self.declare_parameter("depth_percentile", 20.0)

        self.declare_parameter("roi_row_frac_min", 0.22)
        self.declare_parameter("roi_row_frac_max", 0.50)
        self.declare_parameter("roi_col_frac_center_min", 0.38)
        self.declare_parameter("roi_col_frac_center_max", 0.62)
        self.declare_parameter("roi_col_frac_left_min", 0.08)
        self.declare_parameter("roi_col_frac_left_max", 0.38)
        self.declare_parameter("roi_col_frac_right_min", 0.62)
        self.declare_parameter("roi_col_frac_right_max", 0.92)

        self.declare_parameter("debug_avoidance", True)
        self.declare_parameter("debug_avoidance_period_sec", 1.0)

        qos_sensor = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value), self._depth_cb, qos_sensor
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 20
        )
        self._pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / hz, self._tick)

        self._last_depth: Optional[np.ndarray] = None
        self._odom_xy: Optional[Tuple[float, float]] = None
        self._anchor_xy: Optional[Tuple[float, float]] = None
        self._anchor_t = time.monotonic()
        self._odom_progress_m = 0.0

        self._state = _State.FORWARD
        self._reason = "init"
        self._phase_until = 0.0
        self._slow_until = 0.0
        self._turn_t0: Optional[float] = None
        self._turn_dir = 1.0
        self._recovery_dir = 1.0
        self._dbg_last_t = 0.0
        self._warn_no_depth = False

    def _depth_cb(self, msg: Image) -> None:
        d = _decode_depth(msg)
        if d is not None:
            self._last_depth = d

    def _odom_cb(self, msg: Odometry) -> None:
        xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        if self._anchor_xy is None:
            self._anchor_xy = xy
            self._anchor_t = time.monotonic()
        self._odom_xy = xy

    def _tick(self) -> None:
        now = time.monotonic()
        p = self._params()
        self._update_odom_progress(now, p)

        front, left, right, valid = self._measure_regions(p)
        twist = Twist()
        reason = "idle"

        if self._odom_stuck(now, p):
            self._enter_reverse(now, p, "odom_no_progress")
            twist.linear.x = p["v_rev"]
            twist.angular.z = 0.0
            reason = "odom_no_progress"
            self._state = _State.REVERSE
        else:
            twist, reason = self._run_state_machine(now, p, front, left, right, valid)

        self._enforce_cmd(twist, self._state, p)
        self._pub.publish(twist)
        self._log_debug(now, p, twist, front, left, right, reason)

    def _run_state_machine(
        self,
        now: float,
        p: dict,
        front: float,
        left: float,
        right: float,
        valid: bool,
    ) -> Tuple[Twist, str]:
        twist = Twist()

        if self._state == _State.REVERSE:
            if now < self._phase_until:
                twist.linear.x = p["v_rev"]
                return twist, "critical_close"
            self._state = _State.RECOVERY_TURN
            self._phase_until = now + p["recovery_turn_time_s"]
            self._recovery_dir *= -1.0
            twist.angular.z = self._recovery_dir * p["w_turn"]
            return twist, "recovery_turn"

        if self._state == _State.RECOVERY_TURN:
            if now < self._phase_until and not (valid and front > p["safe"]):
                twist.angular.z = self._recovery_dir * p["w_turn"]
                return twist, "recovery_turn"
            self._state = _State.FORWARD
            self._turn_t0 = None
            self._reset_anchor(now)
            if valid and front < p["clear"]:
                return self._enter_slow_forward(now, p, front, left, right)
            if not valid:
                return self._enter_search_turn(p)
            twist.linear.x = p["v_fwd"]
            self._state = _State.FORWARD
            return twist, "clear"

        if not valid:
            return self._enter_search_turn(p)

        if front < p["critical"]:
            self._enter_reverse(now, p, "critical_close")
            twist.linear.x = p["v_rev"]
            return twist, "critical_close"

        if self._state == _State.SLOW_FORWARD:
            if now >= self._slow_until:
                if front >= p["clear"]:
                    self._state = _State.FORWARD
                    twist.linear.x = p["v_fwd"]
                    return twist, "clear"
                if front < p["safe"]:
                    return self._enter_turn(now, p, front, left, right, "front_blocked_turn_to_freer_side")
                self._slow_until = now + p["slow_max_s"]
            twist.linear.x = p["v_slow"]
            twist.angular.z = 0.15 * self._away_sign(left, right) * p["w_turn"]
            return twist, "cautious_forward"

        if self._state in (_State.TURN_LEFT, _State.TURN_RIGHT):
            if self._turn_t0 is None:
                self._turn_t0 = now
            elif (now - self._turn_t0) >= p["stuck_timeout_s"]:
                self._enter_reverse(now, p, "turn_timeout")
                twist.linear.x = p["v_rev"]
                return twist, "turn_timeout"
            if front >= p["safe"]:
                self._turn_t0 = None
                if front >= p["clear"]:
                    self._state = _State.FORWARD
                    twist.linear.x = p["v_fwd"]
                    return twist, "clear"
                return self._enter_slow_forward(now, p, front, left, right)
            twist.angular.z = self._turn_dir * p["w_turn"]
            return twist, "front_blocked_turn_to_freer_side"

        if front >= p["clear"]:
            self._state = _State.FORWARD
            self._turn_t0 = None
            twist.linear.x = p["v_fwd"]
            self._reset_anchor(now)
            return twist, "clear"

        if front >= p["safe"]:
            return self._enter_slow_forward(now, p, front, left, right)

        return self._enter_turn(now, p, front, left, right, "front_blocked_turn_to_freer_side")

    def _enter_slow_forward(
        self, now: float, p: dict, front: float, left: float, right: float
    ) -> Tuple[Twist, str]:
        self._state = _State.SLOW_FORWARD
        self._slow_until = now + p["slow_max_s"]
        t = Twist()
        t.linear.x = p["v_slow"]
        t.angular.z = 0.15 * self._away_sign(left, right) * p["w_turn"]
        return t, "cautious_forward"

    def _enter_turn(
        self,
        now: float,
        p: dict,
        front: float,
        left: float,
        right: float,
        reason: str,
    ) -> Tuple[Twist, str]:
        self._turn_dir = self._freer_sign(left, right)
        self._state = _State.TURN_LEFT if self._turn_dir > 0 else _State.TURN_RIGHT
        self._turn_t0 = now
        t = Twist()
        t.angular.z = self._turn_dir * p["w_turn"]
        return t, reason

    def _enter_search_turn(self, p: dict) -> Tuple[Twist, str]:
        self._state = _State.SEARCH_TURN
        t = Twist()
        t.angular.z = self._turn_dir * p["w_search"]
        return t, "no_valid_depth"

    def _enter_reverse(self, now: float, p: dict, reason: str) -> None:
        self._state = _State.REVERSE
        self._phase_until = now + p["reverse_time_s"]
        self._turn_t0 = None
        self._reset_anchor(now)
        self._reason = reason

    def _params(self) -> dict:
        return {
            "safe": float(self.get_parameter("safe_distance_m").value),
            "critical": float(self.get_parameter("critical_distance_m").value),
            "clear": float(self.get_parameter("clear_distance_m").value),
            "v_fwd": float(self.get_parameter("forward_speed_m_s").value),
            "v_slow": float(self.get_parameter("slow_forward_speed_m_s").value),
            "w_turn": float(self.get_parameter("turn_speed_rad_s").value),
            "w_search": float(self.get_parameter("search_turn_speed_rad_s").value),
            "v_rev": min(float(self.get_parameter("reverse_speed_m_s").value), -1e-6),
            "reverse_time_s": max(float(self.get_parameter("reverse_time_s").value), 0.1),
            "recovery_turn_time_s": max(
                float(self.get_parameter("recovery_turn_time_s").value), 0.3
            ),
            "stuck_timeout_s": max(float(self.get_parameter("stuck_timeout_s").value), 0.5),
            "progress_eps": max(float(self.get_parameter("progress_epsilon_m").value), 1e-4),
            "vmax": float(self.get_parameter("max_range_m").value),
            "pct": float(self.get_parameter("depth_percentile").value),
            "min_vx": float(self.get_parameter("min_effective_linear_speed_m_s").value),
            "min_wz": float(self.get_parameter("min_effective_turn_speed_rad_s").value),
            "slow_max_s": max(float(self.get_parameter("slow_forward_max_s").value), 0.5),
            "rs": float(self.get_parameter("roi_row_frac_min").value),
            "re": float(self.get_parameter("roi_row_frac_max").value),
            "cc0": float(self.get_parameter("roi_col_frac_center_min").value),
            "cc1": float(self.get_parameter("roi_col_frac_center_max").value),
            "lc0": float(self.get_parameter("roi_col_frac_left_min").value),
            "lc1": float(self.get_parameter("roi_col_frac_left_max").value),
            "rc0": float(self.get_parameter("roi_col_frac_right_min").value),
            "rc1": float(self.get_parameter("roi_col_frac_right_max").value),
            "dbg": _as_bool(self.get_parameter("debug_avoidance").value),
            "dbg_period": max(float(self.get_parameter("debug_avoidance_period_sec").value), 0.25),
        }

    def _measure_regions(self, p: dict) -> Tuple[float, float, float, bool]:
        d = self._last_depth
        if d is None:
            if not self._warn_no_depth:
                self._warn_no_depth = True
                self.get_logger().warn("No depth frame yet")
            return float("nan"), float("nan"), float("nan"), False
        h, w = d.shape[:2]
        r0, r1, _, _ = _frac_window(h, w, p["rs"], p["re"], 0.0, 1.0)
        _, _, cc0, cc1 = _frac_window(h, w, p["rs"], p["re"], p["cc0"], p["cc1"])
        _, _, lc0, lc1 = _frac_window(h, w, p["rs"], p["re"], p["lc0"], p["lc1"])
        _, _, rc0, rc1 = _frac_window(h, w, p["rs"], p["re"], p["rc0"], p["rc1"])
        front, nf = _robust_distance(d[r0:r1, cc0:cc1], p["vmax"], p["pct"])
        left, _ = _robust_distance(d[r0:r1, lc0:lc1], p["vmax"], p["pct"])
        right, _ = _robust_distance(d[r0:r1, rc0:rc1], p["vmax"], p["pct"])
        if not np.isfinite(left):
            left = front
        if not np.isfinite(right):
            right = front
        valid = nf > 0 and np.isfinite(front)
        return front, left, right, valid

    def _freer_sign(self, d_left: float, d_right: float) -> float:
        if np.isfinite(d_left) and np.isfinite(d_right):
            if d_left > d_right + 0.04:
                return 1.0
            if d_right > d_left + 0.04:
                return -1.0
        return self._turn_dir

    def _away_sign(self, d_left: float, d_right: float) -> float:
        if np.isfinite(d_left) and np.isfinite(d_right):
            if d_left > d_right:
                return 1.0
            if d_right > d_left:
                return -1.0
        return self._turn_dir

    def _reset_anchor(self, now: float) -> None:
        if self._odom_xy is not None:
            self._anchor_xy = self._odom_xy
            self._anchor_t = now
            self._odom_progress_m = 0.0

    def _update_odom_progress(self, now: float, p: dict) -> None:
        if self._odom_xy is None or self._anchor_xy is None:
            self._odom_progress_m = 0.0
            return
        self._odom_progress_m = math.hypot(
            self._odom_xy[0] - self._anchor_xy[0], self._odom_xy[1] - self._anchor_xy[1]
        )
        if self._odom_progress_m >= p["progress_eps"]:
            self._anchor_xy = self._odom_xy
            self._anchor_t = now
            self._odom_progress_m = 0.0

    def _odom_stuck(self, now: float, p: dict) -> bool:
        if self._state in (_State.REVERSE, _State.RECOVERY_TURN):
            return False
        if self._odom_xy is None or self._anchor_xy is None:
            return False
        if self._odom_progress_m >= p["progress_eps"]:
            return False
        if (now - self._anchor_t) < p["stuck_timeout_s"]:
            return False
        self._recovery_dir *= -1.0
        return True

    def _enforce_cmd(self, twist: Twist, state: _State, p: dict) -> None:
        if state == _State.FORWARD:
            if twist.linear.x > 0.0 and twist.linear.x < p["min_vx"]:
                twist.linear.x = p["min_vx"]
            elif twist.linear.x <= 0.0:
                twist.linear.x = p["min_vx"]
        elif state == _State.SLOW_FORWARD:
            if twist.linear.x > 0.0 and twist.linear.x < p["min_vx"]:
                twist.linear.x = p["min_vx"]
            elif twist.linear.x <= 0.0:
                twist.linear.x = p["min_vx"]
        elif state == _State.REVERSE:
            if twist.linear.x > p["v_rev"] * 0.5:
                twist.linear.x = p["v_rev"]
        elif state in (_State.TURN_LEFT, _State.TURN_RIGHT, _State.RECOVERY_TURN, _State.SEARCH_TURN):
            wz = twist.angular.z
            if abs(wz) < p["min_wz"]:
                sign = 1.0 if wz >= 0.0 else -1.0
                if wz == 0.0:
                    sign = self._turn_dir if state != _State.RECOVERY_TURN else self._recovery_dir
                twist.angular.z = sign * p["min_wz"]

    def _log_debug(
        self,
        now: float,
        p: dict,
        twist: Twist,
        front: float,
        left: float,
        right: float,
        reason: str,
    ) -> None:
        if not p["dbg"] or (now - self._dbg_last_t) < p["dbg_period"]:
            return
        self._dbg_last_t = now
        ff = "%.2f" % front if np.isfinite(front) else "nan"
        lf = "%.2f" % left if np.isfinite(left) else "nan"
        rf = "%.2f" % right if np.isfinite(right) else "nan"
        self.get_logger().info(
            "[avoidance] state=%s front=%s left=%s right=%s cmd_linear=%.3f cmd_angular=%.3f "
            "odom_progress=%.3f reason=%s"
            % (
                self._state.name,
                ff,
                lf,
                rf,
                twist.linear.x,
                twist.angular.z,
                self._odom_progress_m,
                reason,
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
