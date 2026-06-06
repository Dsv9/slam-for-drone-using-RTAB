"""DEMO depth avoidance — 10 Hz timer FSM with WALL_ESCAPE anti-stun-lock.

Publishes /drone/cmd_vel -> cmd_vel_watchdog -> /drone/cmd_vel_safe
  -> gazebo_controller_bridge_node -> /cmd_vel -> ros_gz_bridge -> Gazebo.
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
    WALL_ESCAPE = 6
    SEARCH_TURN = 7


class _WallEscapePhase(IntEnum):
    REVERSE = 0
    TURN = 1
    FORCE_FORWARD = 2


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

        self.declare_parameter("safe_distance_m", 0.45)
        self.declare_parameter("critical_distance_m", 0.25)
        self.declare_parameter("clear_distance_m", 0.75)
        self.declare_parameter("forward_speed_m_s", 0.07)
        self.declare_parameter("slow_forward_speed_m_s", 0.035)
        self.declare_parameter("turn_speed_rad_s", 0.45)
        self.declare_parameter("search_turn_speed_rad_s", 0.35)
        self.declare_parameter("reverse_speed_m_s", -0.06)
        self.declare_parameter("reverse_time_s", 1.2)
        self.declare_parameter("recovery_turn_time_s", 2.0)
        self.declare_parameter("wall_escape_turn_time_s", 2.8)
        self.declare_parameter("wall_escape_reverse_time_s", 1.5)
        self.declare_parameter("force_forward_after_escape_s", 0.8)
        self.declare_parameter("corner_lock_timeout_s", 4.0)
        self.declare_parameter("turn_flip_limit", 3)
        self.declare_parameter("turn_flip_window_s", 6.0)
        self.declare_parameter("side_free_margin_m", 0.05)
        self.declare_parameter("stuck_timeout_s", 2.0)
        self.declare_parameter("force_forward_time_s", 0.8)
        self.declare_parameter("progress_epsilon_m", 0.025)
        self.declare_parameter("max_range_m", 3.0)
        self.declare_parameter("min_effective_linear_speed_m_s", 0.035)
        self.declare_parameter("min_effective_turn_speed_rad_s", 0.30)
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
        self._odom_valid = False
        self._anchor_xy: Optional[Tuple[float, float]] = None
        self._anchor_t = time.monotonic()
        self._odom_progress_m = 0.0

        self._state = _State.FORWARD
        self._state_t0 = time.monotonic()
        self._reason = "init"
        self._phase_until = 0.0
        self._slow_until = 0.0
        self._turn_dir = 1.0
        self._escape_dir = 1.0
        self._wall_phase = _WallEscapePhase.REVERSE
        self._dbg_last_t = 0.0
        self._warn_no_depth = False
        self._non_forward_t0: Optional[float] = None
        self._front_not_clear_t0: Optional[float] = None
        self._turn_flips: list[float] = []

    def _depth_cb(self, msg: Image) -> None:
        d = _decode_depth(msg)
        if d is not None:
            self._last_depth = d

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        if abs(q.w) < 1e-6:
            self._odom_valid = False
            return
        xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        if not (math.isfinite(xy[0]) and math.isfinite(xy[1])):
            self._odom_valid = False
            return
        self._odom_valid = True
        if self._anchor_xy is None:
            self._anchor_xy = xy
            self._anchor_t = time.monotonic()
        self._odom_xy = xy

    def _tick(self) -> None:
        now = time.monotonic()
        p = self._params()
        self._update_odom_progress(now, p)

        front, left, right, valid = self._measure_regions(p)
        self._update_front_blocked_timer(now, p, front, valid)
        twist = Twist()
        reason = "idle"

        corner_lock, lock_reason = self._corner_lock_detected(now, p, front, left, right, valid)

        if self._state == _State.WALL_ESCAPE:
            twist, reason = self._run_wall_escape(now, p, front, valid)
        elif self._state == _State.REVERSE:
            twist, reason = self._run_reverse(now, p)
        elif self._state == _State.RECOVERY_TURN:
            twist, reason = self._run_recovery_turn(now, p)
        elif corner_lock:
            self._enter_wall_escape(now, p, lock_reason)
            twist, reason = self._run_wall_escape(now, p, front, valid)
        elif not valid:
            twist, reason = self._run_search_turn(now, p)
        elif front < p["critical"]:
            self._enter_reverse(now, p, "critical_close_reverse")
            twist, reason = self._run_reverse(now, p)
        else:
            twist, reason = self._run_normal(now, p, front, left, right)

        if (
            self._odom_valid
            and self._state not in (_State.REVERSE, _State.RECOVERY_TURN, _State.WALL_ESCAPE)
            and self._odom_stuck(now, p)
        ):
            self._enter_wall_escape(now, p, "odom_no_progress")
            twist, reason = self._run_wall_escape(now, p, front, valid)

        self._enforce_cmd(twist, self._state, p)
        self._pub.publish(twist)
        self._log_debug(now, p, twist, front, left, right, reason)

    def _state_time(self, now: float) -> float:
        return now - self._state_t0

    def _set_state(self, state: _State, now: float, reason: str) -> None:
        prev = self._state
        if prev != state:
            if (
                (prev == _State.TURN_LEFT and state == _State.TURN_RIGHT)
                or (prev == _State.TURN_RIGHT and state == _State.TURN_LEFT)
            ):
                self._turn_flips.append(now)
            if state == _State.FORWARD:
                self._non_forward_t0 = None
                self._front_not_clear_t0 = None
                self._turn_flips.clear()
            elif state in (
                _State.SLOW_FORWARD,
                _State.TURN_LEFT,
                _State.TURN_RIGHT,
                _State.SEARCH_TURN,
            ):
                if self._non_forward_t0 is None:
                    self._non_forward_t0 = now
            self._state = state
            self._state_t0 = now
        self._reason = reason

    def _update_front_blocked_timer(
        self, now: float, p: dict, front: float, valid: bool
    ) -> None:
        if valid and np.isfinite(front) and front < p["clear"]:
            if self._front_not_clear_t0 is None:
                self._front_not_clear_t0 = now
        else:
            self._front_not_clear_t0 = None

    def _sides_indecisive(self, left: float, right: float, p: dict) -> bool:
        if not (np.isfinite(left) and np.isfinite(right)):
            return False
        margin = p["side_margin"]
        both_blocked = left < p["safe"] and right < p["safe"]
        nearly_equal = abs(left - right) < margin
        both_tight = left < p["clear"] and right < p["clear"]
        return both_blocked or (nearly_equal and both_tight)

    def _corner_lock_detected(
        self,
        now: float,
        p: dict,
        front: float,
        left: float,
        right: float,
        valid: bool,
    ) -> Tuple[bool, str]:
        if self._state in (
            _State.WALL_ESCAPE,
            _State.REVERSE,
            _State.RECOVERY_TURN,
            _State.FORWARD,
        ):
            return False, ""

        window = p["turn_flip_window_s"]
        self._turn_flips = [t for t in self._turn_flips if (now - t) <= window]
        if len(self._turn_flips) > p["turn_flip_limit"]:
            return True, "turn_flip_lock"

        blocked_elapsed = (
            (now - self._non_forward_t0)
            if self._non_forward_t0 is not None
            else 0.0
        )
        if blocked_elapsed >= p["corner_lock_timeout_s"]:
            return True, "corner_lock_timeout"

        if (
            valid
            and self._front_not_clear_t0 is not None
            and (now - self._front_not_clear_t0) >= p["corner_lock_timeout_s"]
        ):
            return True, "corner_lock_timeout"

        if (
            valid
            and self._sides_indecisive(left, right, p)
            and self._state in (_State.TURN_LEFT, _State.TURN_RIGHT, _State.SLOW_FORWARD)
            and blocked_elapsed >= p["corner_lock_timeout_s"] * 0.5
        ):
            return True, "corner_lock_timeout"

        return False, ""

    def _reset_corner_lock_trackers(self) -> None:
        self._non_forward_t0 = None
        self._front_not_clear_t0 = None
        self._turn_flips.clear()

    def _run_normal(
        self, now: float, p: dict, front: float, left: float, right: float
    ) -> Tuple[Twist, str]:
        twist = Twist()

        if self._state == _State.SLOW_FORWARD:
            if self._state_time(now) >= p["slow_max_s"]:
                if front >= p["clear"]:
                    self._set_state(_State.FORWARD, now, "clear_forward")
                    twist.linear.x = p["v_fwd"]
                    return twist, "clear_forward"
                if front < p["safe"]:
                    return self._enter_turn(now, p, left, right, "front_blocked_turn_to_freer_side")
            twist.linear.x = p["v_slow"]
            twist.angular.z = 0.15 * self._away_sign(left, right)
            return twist, "cautious_forward"

        if self._state in (_State.TURN_LEFT, _State.TURN_RIGHT):
            if front >= p["clear"]:
                self._set_state(_State.FORWARD, now, "clear_forward")
                twist.linear.x = p["v_fwd"]
                return twist, "clear_forward"
            if front >= p["safe"]:
                return self._enter_slow_forward(now, p, left, right)
            twist.angular.z = self._turn_dir * p["w_turn"]
            return twist, "front_blocked_turn_to_freer_side"

        if self._state == _State.SEARCH_TURN:
            twist.angular.z = p["w_search"]
            return twist, "no_valid_depth"

        if front >= p["clear"]:
            self._set_state(_State.FORWARD, now, "clear_forward")
            twist.linear.x = p["v_fwd"]
            return twist, "clear_forward"

        if front >= p["safe"]:
            return self._enter_slow_forward(now, p, left, right)

        return self._enter_turn(now, p, left, right, "front_blocked_turn_to_freer_side")

    def _run_reverse(self, now: float, p: dict) -> Tuple[Twist, str]:
        twist = Twist()
        if now < self._phase_until:
            twist.linear.x = p["v_rev"]
            return twist, "critical_close_reverse"
        self._set_state(_State.RECOVERY_TURN, now, "recovery_turn_after_reverse")
        self._phase_until = now + p["recovery_turn_time_s"]
        self._escape_dir *= -1.0
        twist.angular.z = self._escape_dir * p["w_turn"]
        return twist, "recovery_turn_after_reverse"

    def _run_recovery_turn(self, now: float, p: dict) -> Tuple[Twist, str]:
        twist = Twist()
        if now < self._phase_until:
            twist.angular.z = self._escape_dir * p["w_turn"]
            return twist, "recovery_turn_after_reverse"
        self._set_state(_State.FORWARD, now, "recovery_done")
        twist.linear.x = p["v_fwd"]
        return twist, "recovery_done"

    def _run_search_turn(self, now: float, p: dict) -> Tuple[Twist, str]:
        if self._state != _State.SEARCH_TURN:
            self._set_state(_State.SEARCH_TURN, now, "no_valid_depth")
        twist = Twist()
        twist.angular.z = p["w_search"]
        return twist, "no_valid_depth"

    def _enter_wall_escape(self, now: float, p: dict, reason: str = "wall_corner_escape") -> None:
        self._wall_phase = _WallEscapePhase.REVERSE
        self._phase_until = now + p["wall_escape_reverse_time_s"]
        self._escape_dir *= -1.0
        self._reset_corner_lock_trackers()
        self._set_state(_State.WALL_ESCAPE, now, reason)

    def _run_wall_escape(
        self, now: float, p: dict, front: float, valid: bool
    ) -> Tuple[Twist, str]:
        twist = Twist()

        if self._wall_phase == _WallEscapePhase.REVERSE:
            if now < self._phase_until:
                twist.linear.x = p["v_rev"]
                return twist, "wall_escape_reverse"
            self._wall_phase = _WallEscapePhase.TURN
            self._phase_until = now + p["wall_escape_turn_time_s"]
            twist.angular.z = self._escape_dir * p["w_turn"]
            return twist, "wall_escape_turn"

        if self._wall_phase == _WallEscapePhase.TURN:
            if now < self._phase_until:
                twist.angular.z = self._escape_dir * p["w_turn"]
                return twist, "wall_escape_turn"
            self._wall_phase = _WallEscapePhase.FORCE_FORWARD
            self._phase_until = now + p["force_forward_after_escape_s"]
            if valid and front > p["critical"]:
                twist.linear.x = p["v_fwd"]
                return twist, "force_forward_after_escape"
            self._wall_phase = _WallEscapePhase.REVERSE
            self._phase_until = now + p["wall_escape_reverse_time_s"]
            twist.linear.x = p["v_rev"]
            return twist, "wall_escape_reverse"

        if now < self._phase_until:
            if valid and front <= p["critical"]:
                self._wall_phase = _WallEscapePhase.REVERSE
                self._phase_until = now + p["wall_escape_reverse_time_s"]
                twist.linear.x = p["v_rev"]
                return twist, "wall_escape_reverse"
            twist.linear.x = p["v_fwd"]
            return twist, "force_forward_after_escape"

        self._set_state(_State.FORWARD, now, "wall_escape_done")
        self._reset_corner_lock_trackers()
        twist.linear.x = p["v_fwd"]
        return twist, "wall_escape_done"

    def _enter_slow_forward(
        self, now: float, p: dict, left: float, right: float
    ) -> Tuple[Twist, str]:
        self._set_state(_State.SLOW_FORWARD, now, "cautious_forward")
        self._slow_until = now + p["slow_max_s"]
        t = Twist()
        t.linear.x = p["v_slow"]
        t.angular.z = 0.15 * self._away_sign(left, right)
        return t, "cautious_forward"

    def _enter_turn(
        self, now: float, p: dict, left: float, right: float, reason: str
    ) -> Tuple[Twist, str]:
        self._turn_dir = self._freer_sign(left, right)
        st = _State.TURN_LEFT if self._turn_dir > 0 else _State.TURN_RIGHT
        self._set_state(st, now, reason)
        t = Twist()
        t.angular.z = self._turn_dir * p["w_turn"]
        return t, reason

    def _enter_reverse(self, now: float, p: dict, reason: str) -> None:
        self._set_state(_State.REVERSE, now, reason)
        self._phase_until = now + p["reverse_time_s"]

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
            "wall_escape_turn_time_s": max(
                float(self.get_parameter("wall_escape_turn_time_s").value), 0.5
            ),
            "wall_escape_reverse_time_s": max(
                float(self.get_parameter("wall_escape_reverse_time_s").value), 0.3
            ),
            "force_forward_after_escape_s": max(
                float(self.get_parameter("force_forward_after_escape_s").value), 0.3
            ),
            "corner_lock_timeout_s": max(
                float(self.get_parameter("corner_lock_timeout_s").value), 1.0
            ),
            "turn_flip_limit": int(self.get_parameter("turn_flip_limit").value),
            "turn_flip_window_s": max(
                float(self.get_parameter("turn_flip_window_s").value), 1.0
            ),
            "side_margin": float(self.get_parameter("side_free_margin_m").value),
            "force_forward_time_s": max(
                float(self.get_parameter("force_forward_time_s").value), 0.3
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
            "demo": _as_bool(self.get_parameter("demo_avoidance_mode").value),
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
        return self._turn_dir if self._turn_dir != 0.0 else self._escape_dir

    def _away_sign(self, d_left: float, d_right: float) -> float:
        if np.isfinite(d_left) and np.isfinite(d_right):
            if d_left > d_right:
                return 1.0
            if d_right > d_left:
                return -1.0
        return self._escape_dir

    def _update_odom_progress(self, now: float, p: dict) -> None:
        if not self._odom_valid or self._odom_xy is None or self._anchor_xy is None:
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
        if self._odom_xy is None or self._anchor_xy is None:
            return False
        if self._odom_progress_m >= p["progress_eps"]:
            return False
        return (now - self._anchor_t) >= p["stuck_timeout_s"]

    def _enforce_cmd(self, twist: Twist, state: _State, p: dict) -> None:
        if state == _State.FORWARD:
            if twist.linear.x <= 0.0:
                twist.linear.x = p["v_fwd"]
            elif twist.linear.x < p["v_fwd"]:
                twist.linear.x = p["v_fwd"]
        elif state == _State.SLOW_FORWARD:
            floor = max(p["v_slow"], p["min_vx"])
            if twist.linear.x <= 0.0:
                twist.linear.x = floor
            elif twist.linear.x < floor:
                twist.linear.x = floor
        elif state == _State.REVERSE:
            if twist.linear.x > p["v_rev"] * 0.5:
                twist.linear.x = p["v_rev"]
        elif state == _State.WALL_ESCAPE:
            if twist.linear.x < 0.0:
                twist.linear.x = p["v_rev"]
            elif twist.linear.x > 0.0:
                if twist.linear.x < p["v_fwd"]:
                    twist.linear.x = p["v_fwd"]
            if abs(twist.angular.z) > 1e-6:
                sign = 1.0 if twist.angular.z >= 0.0 else -1.0
                twist.angular.z = sign * p["w_turn"]
        elif state in (
            _State.TURN_LEFT,
            _State.TURN_RIGHT,
            _State.RECOVERY_TURN,
            _State.SEARCH_TURN,
        ):
            wz = twist.angular.z
            floor = p["min_wz"]
            if abs(wz) < floor:
                sign = 1.0 if wz >= 0.0 else -1.0
                if wz == 0.0:
                    sign = self._turn_dir if state in (_State.TURN_LEFT, _State.TURN_RIGHT) else self._escape_dir
                twist.angular.z = sign * max(floor, p["w_turn"] if state != _State.SEARCH_TURN else p["w_search"])
            elif state != _State.SEARCH_TURN and abs(wz) < p["w_turn"]:
                sign = 1.0 if wz >= 0.0 else -1.0
                twist.angular.z = sign * p["w_turn"]

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
        mode = "DEMO" if p["demo"] else "LEGACY"
        self.get_logger().info(
            "[avoidance] mode=%s state=%s front=%s left=%s right=%s "
            "cmd_linear=%.3f cmd_angular=%.3f state_time=%.1f reason=%s"
            % (
                mode,
                self._state.name,
                ff,
                lf,
                rf,
                twist.linear.x,
                twist.angular.z,
                self._state_time(now),
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
