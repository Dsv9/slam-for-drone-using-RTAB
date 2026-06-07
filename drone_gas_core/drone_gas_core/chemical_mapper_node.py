import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
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


class ChemicalMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("chemical_mapper_node")
        self.declare_parameter("pose_topic", "/rtabmap/localization_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gas_topic", "/gas/concentration")
        self.declare_parameter("map_topic", "/gas/chemical_map")
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("resolution", 0.2)
        self.declare_parameter("width", 200)
        self.declare_parameter("height", 200)
        self.declare_parameter("origin_x", -20.0)
        self.declare_parameter("origin_y", -20.0)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("splat_radius_m", 0.8)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("gas_max", 2.0)
        self.declare_parameter("gas_pose_fallback_mode", True)
        self.declare_parameter("fallback_x", 0.0)
        self.declare_parameter("fallback_y", 0.0)

        self.res = float(self.get_parameter("resolution").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.ox = float(self.get_parameter("origin_x").value)
        self.oy = float(self.get_parameter("origin_y").value)
        self.alpha = float(self.get_parameter("ema_alpha").value)
        self.splat_r = float(self.get_parameter("splat_radius_m").value)
        self.gas_max = float(self.get_parameter("gas_max").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.grid = np.full((self.h, self.w), -1.0, dtype=np.float32)

        self._rtabmap_xy: Optional[Tuple[float, float]] = None
        self._odom_xy: Optional[Tuple[float, float]] = None
        self._rtabmap_valid = False
        self._odom_valid = False
        self._gas: Optional[float] = None

        self.pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("map_topic").value), 10
        )
        self.create_subscription(
            PoseStamped, str(self.get_parameter("pose_topic").value), self._pose_cb, 30
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 30
        )
        self.create_subscription(
            Float32, str(self.get_parameter("gas_topic").value), self._gas_cb, 30
        )
        hz = max(float(self.get_parameter("publish_hz").value), 0.5)
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
        self._rtabmap_xy = (float(p.x), float(p.y))
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
        self._odom_xy = (float(p.x), float(p.y))
        self._odom_valid = True

    def _gas_cb(self, msg: Float32) -> None:
        self._gas = float(msg.data)

    def _select_xy(self) -> Optional[Tuple[float, float]]:
        if self._rtabmap_valid and self._rtabmap_xy is not None:
            return self._rtabmap_xy
        if self._odom_valid and self._odom_xy is not None:
            return self._odom_xy
        if _as_bool(self.get_parameter("gas_pose_fallback_mode").value):
            return (
                float(self.get_parameter("fallback_x").value),
                float(self.get_parameter("fallback_y").value),
            )
        return None

    def world_to_cell(self, x: float, y: float):
        cx = int(math.floor((x - self.ox) / self.res))
        cy = int(math.floor((y - self.oy) / self.res))
        return (cx, cy) if 0 <= cx < self.w and 0 <= cy < self.h else None

    def _splat(self, xy: Tuple[float, float], gas: float) -> None:
        c = self.world_to_cell(xy[0], xy[1])
        if c is None:
            return
        cx, cy = c
        r = max(1, int(self.splat_r / self.res))
        g = 100.0 * max(0.0, min(gas, self.gas_max)) / max(self.gas_max, 1e-6)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.w and 0 <= ny < self.h):
                    continue
                d = math.sqrt(dx * dx + dy * dy) * self.res
                if d > self.splat_r:
                    continue
                val = g * math.exp(-0.5 * (d / max(self.splat_r * 0.5, 1e-3)) ** 2)
                old = self.grid[ny, nx]
                self.grid[ny, nx] = (
                    val if old < 0 else (1 - self.alpha) * old + self.alpha * val
                )

    def _tick(self) -> None:
        if self._gas is not None:
            xy = self._select_xy()
            if xy is not None:
                self._splat(xy, self._gas)
        self._publish_map()

    def _publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        info = MapMetaData()
        info.resolution = self.res
        info.width = self.w
        info.height = self.h
        info.origin.position.x = self.ox
        info.origin.position.y = self.oy
        info.origin.orientation.w = 1.0
        msg.info = info
        data = np.where(self.grid < 0, -1, np.clip(np.round(self.grid), 0, 100)).astype(
            np.int8
        )
        msg.data = data.flatten(order="C").tolist()
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ChemicalMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
