import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry


class ExplorationControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("exploration_controller_node")
        self.declare_parameter("pose_topic", "/rtabmap/localization_pose")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("chemical_map_topic", "/gas/chemical_map")
        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("target_altitude", 1.5)
        self.declare_parameter("base_speed", 0.25)
        self.declare_parameter("seek_gain", 0.8)
        self.declare_parameter("max_xy_speed", 0.6)
        self.declare_parameter("max_z_speed", 0.4)
        self.declare_parameter("yaw_rate", 0.0)
        self.pose = None
        self.map_msg = None
        self.phase = 0.0
        self.create_subscription(
            PoseStamped, self.get_parameter("pose_topic").value, self.pose_cb, 20
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_cb, 20
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("chemical_map_topic").value,
            self.map_cb,
            10,
        )
        self.pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 20)
        hz = float(self.get_parameter("control_hz").value)
        self.create_timer(1.0 / max(hz, 1e-3), self.tick)

    def pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def grad(self, x: float, y: float) -> np.ndarray:
        if self.map_msg is None:
            return np.array([0.0, 0.0], dtype=float)
        info = self.map_msg.info
        res = float(info.resolution)
        w, h = int(info.width), int(info.height)
        ox, oy = float(info.origin.position.x), float(info.origin.position.y)
        cx, cy = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
        if cx < 1 or cy < 1 or cx >= w - 1 or cy >= h - 1:
            return np.array([0.0, 0.0], dtype=float)
        grid = np.array(self.map_msg.data, dtype=np.int16).reshape((h, w))
        def v(ix: int, iy: int) -> float:
            a = grid[iy, ix]
            return 0.0 if a < 0 else float(a)
        return np.array(
            [
                (v(cx + 1, cy) - v(cx - 1, cy)) / (2 * res),
                (v(cx, cy + 1) - v(cx, cy - 1)) / (2 * res),
            ],
            dtype=float,
        )

    def tick(self) -> None:
        if self.pose is None:
            return
        x, y, z = self.pose
        self.phase += 0.05
        base_speed = float(self.get_parameter("base_speed").value)
        seek_gain = float(self.get_parameter("seek_gain").value)
        max_xy = float(self.get_parameter("max_xy_speed").value)
        max_z = float(self.get_parameter("max_z_speed").value)
        target_alt = float(self.get_parameter("target_altitude").value)
        vx, vy = base_speed * math.cos(self.phase), base_speed * math.sin(self.phase)
        g = self.grad(x, y)
        n = np.linalg.norm(g)
        if n > 1e-6:
            g = g / n
            vx += seek_gain * g[0]
            vy += seek_gain * g[1]
        sp = math.sqrt(vx * vx + vy * vy)
        if sp > max_xy:
            s = max_xy / sp
            vx, vy = vx * s, vy * s
        vz = max(-max_z, min(max_z, 1.2 * (target_alt - z)))
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        cmd.angular.z = float(self.get_parameter("yaw_rate").value)
        self.pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = ExplorationControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
