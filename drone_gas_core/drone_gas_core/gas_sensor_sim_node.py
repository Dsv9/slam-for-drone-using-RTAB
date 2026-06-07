import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


def _as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    return str(v).lower() in ("true", "1", "yes")


def _valid_position(x: float, y: float, z: float) -> bool:
    return math.isfinite(x) and math.isfinite(y) and math.isfinite(z)


def _valid_quaternion(x: float, y: float, z: float, w: float) -> bool:
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        return False
    if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9 and abs(w) < 1e-9:
        return False
    return True


def _odom_covariance_invalid(msg: Odometry) -> bool:
    cov = msg.pose.covariance
    if len(cov) < 36:
        return False
    high = sum(1 for i in (0, 7, 14, 21, 28, 35) if cov[i] >= 9999.0)
    return high >= 4


class GasSensorSimNode(Node):
    def __init__(self) -> None:
        super().__init__("gas_sensor_sim_node")
        self.declare_parameter("enable_gas", True)
        self.declare_parameter("debug_gas", True)
        self.declare_parameter("pose_topic", "/rtabmap/localization_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gas_topic", "/gas/concentration")
        self.declare_parameter("gas_publish_rate_hz", 5.0)
        self.declare_parameter("gas_pose_fallback_mode", True)
        self.declare_parameter("fallback_x", 0.0)
        self.declare_parameter("fallback_y", 0.0)
        self.declare_parameter("gas_source_x", 2.0)
        self.declare_parameter("gas_source_y", 2.0)
        self.declare_parameter("gas_sigma", 1.5)
        self.declare_parameter("gas_amplitude", 1.0)
        self.declare_parameter("background", 0.02)

        self._rtabmap_pose: Optional[Tuple[float, float, float]] = None
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._rtabmap_valid = False
        self._odom_valid = False
        self._dbg_last_t = 0.0

        topic = str(self.get_parameter("gas_topic").value)
        self._pub = self.create_publisher(Float32, topic, 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._pose_cb,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_cb,
            20,
        )
        hz = max(float(self.get_parameter("gas_publish_rate_hz").value), 1.0)
        self.create_timer(1.0 / hz, self._tick)

    def _pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        if not _valid_position(p.x, p.y, p.z):
            self._rtabmap_valid = False
            return
        if not _valid_quaternion(q.x, q.y, q.z, q.w):
            self._rtabmap_valid = False
            return
        self._rtabmap_pose = (float(p.x), float(p.y), float(p.z))
        self._rtabmap_valid = True

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not _valid_position(p.x, p.y, p.z):
            self._odom_valid = False
            return
        if not _valid_quaternion(q.x, q.y, q.z, q.w):
            self._odom_valid = False
            return
        if _odom_covariance_invalid(msg):
            self._odom_valid = False
            return
        self._odom_pose = (float(p.x), float(p.y), float(p.z))
        self._odom_valid = True

    def _select_pose(self) -> Tuple[Tuple[float, float], str]:
        if self._rtabmap_valid and self._rtabmap_pose is not None:
            x, y, _ = self._rtabmap_pose
            return (x, y), "rtabmap_pose"
        if self._odom_valid and self._odom_pose is not None:
            x, y, _ = self._odom_pose
            return (x, y), "odom"
        if _as_bool(self.get_parameter("gas_pose_fallback_mode").value):
            return (
                float(self.get_parameter("fallback_x").value),
                float(self.get_parameter("fallback_y").value),
            ), "fallback"
        return (0.0, 0.0), "none"

    def _concentration(self, x: float, y: float) -> float:
        sx = float(self.get_parameter("gas_source_x").value)
        sy = float(self.get_parameter("gas_source_y").value)
        sigma = max(float(self.get_parameter("gas_sigma").value), 1e-3)
        amp = float(self.get_parameter("gas_amplitude").value)
        bg = float(self.get_parameter("background").value)
        dist_sq = (x - sx) ** 2 + (y - sy) ** 2
        return bg + amp * math.exp(-dist_sq / (2.0 * sigma * sigma))

    def _tick(self) -> None:
        if not _as_bool(self.get_parameter("enable_gas").value):
            return
        xy, source = self._select_pose()
        c = self._concentration(xy[0], xy[1])
        self._pub.publish(Float32(data=float(c)))
        self._log_debug(xy, source, c)

    def _log_debug(self, xy: Tuple[float, float], source: str, c: float) -> None:
        if not _as_bool(self.get_parameter("debug_gas").value):
            return
        now = time.monotonic()
        if (now - self._dbg_last_t) < 1.0:
            return
        self._dbg_last_t = now
        self.get_logger().info(
            "[gas] source=%s x=%.2f y=%.2f concentration=%.4f publishing=true"
            % (source, xy[0], xy[1], c)
        )


def main() -> None:
    rclpy.init()
    node = GasSensorSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
