import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Float32


class ChemicalMapperNode(Node):
    def __init__(self):
        super().__init__("chemical_mapper_node")
        self.declare_parameter("pose_topic", "/rtabmap/localization_pose")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("gas_topic", "/gas/concentration")
        self.declare_parameter("map_topic", "/gas/chemical_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("resolution", 0.2)
        self.declare_parameter("width", 200)
        self.declare_parameter("height", 200)
        self.declare_parameter("origin_x", -20.0)
        self.declare_parameter("origin_y", -20.0)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("splat_radius_m", 0.8)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("gas_max", 2.0)
        self.res = float(self.get_parameter("resolution").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.ox = float(self.get_parameter("origin_x").value)
        self.oy = float(self.get_parameter("origin_y").value)
        self.alpha = float(self.get_parameter("ema_alpha").value)
        self.splat_r = float(self.get_parameter("splat_radius_m").value)
        self.gas_max = float(self.get_parameter("gas_max").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.grid = np.full((self.h, self.w), -1.0, dtype=np.float32)
        self.xy = None
        self.gas = None
        self.pub = self.create_publisher(OccupancyGrid, self.get_parameter("map_topic").value, 10)
        self.create_subscription(PoseStamped, self.get_parameter("pose_topic").value, self.pose_cb, 30)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_cb, 30)
        self.create_subscription(Float32, self.get_parameter("gas_topic").value, self.gas_cb, 30)
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 1e-3), self.publish_map)

    def pose_cb(self, msg):
        p = msg.pose.position
        self.xy = (p.x, p.y)
        self.update()

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.xy = (p.x, p.y)
        self.update()

    def gas_cb(self, msg):
        self.gas = float(msg.data)
        self.update()

    def world_to_cell(self, x, y):
        cx, cy = int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))
        return (cx, cy) if 0 <= cx < self.w and 0 <= cy < self.h else None

    def update(self):
        if self.xy is None or self.gas is None:
            return
        c = self.world_to_cell(self.xy[0], self.xy[1])
        if c is None:
            return
        cx, cy = c
        r = max(1, int(self.splat_r / self.res))
        g = 100.0 * max(0.0, min(self.gas, self.gas_max)) / max(self.gas_max, 1e-6)
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
                self.grid[ny, nx] = val if old < 0 else (1 - self.alpha) * old + self.alpha * val

    def publish_map(self):
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
        data = np.where(self.grid < 0, -1, np.clip(np.round(self.grid), 0, 100)).astype(np.int8)
        msg.data = data.flatten(order="C").tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ChemicalMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
