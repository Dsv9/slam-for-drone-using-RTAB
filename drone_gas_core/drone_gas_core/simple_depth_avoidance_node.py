"""Timer-based depth avoidance: left / front / right ROIs, recovery state machine.

Subscribes: /rgbd_camera/depth_image, /odom
Publishes: /drone/cmd_vel (10 Hz) -> cmd_vel_watchdog -> Gazebo /cmd_vel
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
        self.declare_parameter("depth_topic", "/rgbd_camera/depth_image")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("publish_hz", 10.0)

        self.declare_parameter("safe_distance_m", 0.30)
        self.declare_parameter("critical_distance_m", 0.18)
        self.declare_parameter("clear_distance_m", 0.45)
        self.declare_parameter("forward_speed_m_s", 0.025)
        self.declare_parameter("slow_forward_speed_m_s", 0.012)
        self.declare_parameter("turn_speed_rad_s", 0.18)
        self.declare_parameter("search_turn_speed_rad_s", 0.12)
        self.declare_parameter("reverse_speed_m_s", -0.010)
        self.declare_parameter("reverse_time_s", 1.0)
        self.declare_parameter("stuck_timeout_s", 3.0)
        self.declare_parameter("progress_epsilon_m", 0.03)
        self.declare_parameter("max_range_m", 3.0)
        self.declare_parameter("recovery_turn_time_s", 1.5)
        self.declare_parameter("distance_hysteresis_m", 0.06)
        self.declare_parameter("slow_steer_gain", 0.40)
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
        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self.create_subscription(Image, self._depth_topic, self._depth_cb, qos_sensor)
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
        self._progress_m = 0.0

        self._state = _State.FORWARD
        self._reason = "init"
        self._phase_until = 0.0
        self._recovery_wz = 0.0
        self._turn_sign = 1.0
        self._turn_phase_t0: Optional[float] = None
        self._last_cmd_mag = 0.0
        self._dbg_last_t = 0.0
        self._warn_no_depth = False

        self._band_turn = False
        self._band_slow = False

    def _depth_cb(self, msg: Image) -> None:
        d = _decode_depth(msg)
        if d is not None:
            self._last_depth = d

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        xy = (float(p.x), float(p.y))
        if self._anchor_xy is None:
            self._anchor_xy = xy
            self._anchor_t = time.monotonic()
        self._odom_xy = xy

    def _tick(self) -> None:
        now = time.monotonic()
        p = self._params()
        self._update_progress(now, p)

        front, left, right, valid = self._measure_regions(p)
        twist, reason = self._control_step(now, p, front, left, right, valid)
        cmd_mag = abs(twist.linear.x) + abs(twist.angular.z)

        if self._should_recover_stuck(now, p, cmd_mag, front, left, right):
            twist.linear.x = p["v_rev"]
            twist.angular.z = 0.0
            reason = "position_stuck_recovery"

        self._last_cmd_mag = cmd_mag
        self._publish_and_log(now, p, twist, front, left, right, reason)

    def _control_step(
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
                return twist, "critical_reverse"
            self._state = _State.RECOVERY_TURN
            self._phase_until = now + p["recovery_turn_time_s"]
            self._recovery_wz = self._freer_wz(p, left, right)
            self._turn_phase_t0 = None

        if self._state == _State.RECOVERY_TURN:
            if now < self._phase_until and not (valid and front > p["safe"]):
                twist.angular.z = self._recovery_wz
                return twist, "recovery_turn"
            self._state = _State.SLOW_FORWARD if valid and front > p["critical"] else _State.FORWARD
            self._turn_phase_t0 = None
            self._reset_anchor(now)

        if not valid:
            self._state = _State.SEARCH_TURN
            self._band_turn = False
            self._band_slow = False
            twist.angular.z = p["w_search"] * self._turn_sign
            return twist, "no_valid_depth"

        self._update_bands(front, p)

        if front < p["critical"]:
            self._start_reverse(now, p, left, right)
            twist.linear.x = p["v_rev"]
            return twist, "critical_too_close_reverse"

        if self._band_turn or front < p["safe"]:
            self._turn_sign = self._freer_sign(left, right)
            self._state = _State.TURN_LEFT if self._turn_sign > 0 else _State.TURN_RIGHT
            if self._turn_phase_t0 is None:
                self._turn_phase_t0 = now
            elif (now - self._turn_phase_t0) >= p["stuck_timeout_s"]:
                self._turn_sign *= -1.0
                self._start_reverse(now, p, left, right)
                twist.linear.x = p["v_rev"]
                return twist, "turn_timeout_recovery"
            twist.angular.z = self._turn_sign * p["w_turn"]
            return twist, "front_blocked_turn_to_freer_side"

        self._turn_phase_t0 = None

        if self._band_slow or front < p["clear"]:
            self._state = _State.SLOW_FORWARD
            twist.linear.x = p["v_slow"]
            twist.angular.z = self._slow_steer(p, left, right)
            return twist, "cautious_forward"

        self._state = _State.FORWARD
        self._band_slow = False
        self._band_turn = False
        twist.linear.x = p["v_fwd"]
        self._reset_anchor(now)
        return twist, "clear_forward"

    def _params(self) -> dict:
        return {
            "safe": float(self.get_parameter("safe_distance_m").value),
            "critical": float(self.get_parameter("critical_distance_m").value),
            "clear": float(self.get_parameter("clear_distance_m").value),
            "hyst": float(self.get_parameter("distance_hysteresis_m").value),
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
            "slow_steer_gain": float(self.get_parameter("slow_steer_gain").value),
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
                self.get_logger().warn("No depth on %s yet" % self._depth_topic)
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

    def _update_bands(self, front: float, p: dict) -> None:
        h = p["hyst"]
        if self._band_turn:
            if front > p["safe"] + h:
                self._band_turn = False
        elif front < p["safe"]:
            self._band_turn = True

        if self._band_slow:
            if front > p["clear"] + h:
                self._band_slow = False
        elif front < p["clear"]:
            self._band_slow = True

    def _freer_sign(self, d_left: float, d_right: float) -> float:
        if np.isfinite(d_left) and np.isfinite(d_right):
            if d_left > d_right + 0.04:
                return 1.0
            if d_right > d_left + 0.04:
                return -1.0
        return self._turn_sign

    def _freer_wz(self, p: dict, d_left: float, d_right: float) -> float:
        return self._freer_sign(d_left, d_right) * p["w_turn"]

    def _slow_steer(self, p: dict, d_left: float, d_right: float) -> float:
        if not (np.isfinite(d_left) and np.isfinite(d_right)):
            return 0.0
        err = float(np.clip(d_right - d_left, -1.0, 1.0))
        return float(np.clip(err * p["slow_steer_gain"] * p["w_turn"], -p["w_turn"], p["w_turn"]))

    def _reset_anchor(self, now: float) -> None:
        if self._odom_xy is not None:
            self._anchor_xy = self._odom_xy
            self._anchor_t = now
            self._progress_m = 0.0

    def _update_progress(self, now: float, p: dict) -> None:
        if self._odom_xy is None or self._anchor_xy is None:
            self._progress_m = 0.0
            return
        self._progress_m = math.hypot(
            self._odom_xy[0] - self._anchor_xy[0], self._odom_xy[1] - self._anchor_xy[1]
        )
        if self._progress_m >= p["progress_eps"]:
            self._anchor_xy = self._odom_xy
            self._anchor_t = now
            self._progress_m = 0.0

    def _should_recover_stuck(
        self,
        now: float,
        p: dict,
        cmd_mag: float,
        front: float,
        left: float,
        right: float,
    ) -> bool:
        if self._state in (_State.REVERSE, _State.RECOVERY_TURN):
            return False
        if self._odom_xy is None or self._anchor_xy is None:
            return False
        if cmd_mag < 0.02:
            return False
        if self._progress_m >= p["progress_eps"]:
            return False
        if (now - self._anchor_t) < p["stuck_timeout_s"]:
            return False
        self._turn_sign *= -1.0
        self._start_reverse(now, p, left, right)
        return True

    def _start_reverse(self, now: float, p: dict, left: float, right: float) -> None:
        self._state = _State.REVERSE
        self._phase_until = now + p["reverse_time_s"]
        self._recovery_wz = self._freer_wz(p, left, right)
        self._turn_phase_t0 = None
        self._reset_anchor(now)

    def _publish_and_log(
        self,
        now: float,
        p: dict,
        twist: Twist,
        front: float,
        left: float,
        right: float,
        reason: str,
    ) -> None:
        self._reason = reason
        self._pub.publish(twist)
        if not p["dbg"] or (now - self._dbg_last_t) < p["dbg_period"]:
            return
        self._dbg_last_t = now
        ff = "%.2f" % front if np.isfinite(front) else "nan"
        lf = "%.2f" % left if np.isfinite(left) else "nan"
        rf = "%.2f" % right if np.isfinite(right) else "nan"
        self.get_logger().info(
            "[avoidance] state=%s front=%s left=%s right=%s "
            "cmd_linear=%.3f cmd_angular=%.3f progress=%.3f reason=%s"
            % (
                self._state.name,
                ff,
                lf,
                rf,
                twist.linear.x,
                twist.angular.z,
                self._progress_m,
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
