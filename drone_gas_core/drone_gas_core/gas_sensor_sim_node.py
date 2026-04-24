import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class GasSensorSimNode(Node):
    def __init__(self):
        super().__init__("gas_sensor_sim_node")
        self.declare_parameter("pose_topic", "/rtabmap/localization_pose")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("gas_topic", "/gas/concentration")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("noise_std", 0.01)
        self.declare_parameter("background", 0.02)
        self.declare_parameter("source_xy", [2.0, 2.0, -3.0, 1.0])
        self.declare_parameter("source_z", [1.2, 1.0])
        self.declare_parameter("source_strength", [1.0, 0.8])
        self.declare_parameter("sigma_xy", [1.2, 1.8])
        self.declare_parameter("sigma_z", [0.6, 0.7])
        self.pose = None
        self.pub = self.create_publisher(Float32, self.get_parameter("gas_topic").value, 20)
        self.create_subscription(PoseStamped, self.get_parameter("pose_topic").value, self.pose_cb, 20)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_cb, 20)
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 1e-3), self.tick)

    def pose_cb(self, msg):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def tick(self):
        if self.pose is None:
            return
        x, y, z = self.pose
        xy = list(self.get_parameter("source_xy").value)
        sz = list(self.get_parameter("source_z").value)
        ss = list(self.get_parameter("source_strength").value)
        sxy = list(self.get_parameter("sigma_xy").value)
        szz = list(self.get_parameter("sigma_z").value)
        n = min(len(xy) // 2, len(sz), len(ss), len(sxy), len(szz))
        c = float(self.get_parameter("background").value)
        for i in range(n):
            sx, sy = float(xy[2 * i]), float(xy[2 * i + 1])
            c += float(ss[i]) * math.exp(-0.5 * (((x - sx) ** 2 + (y - sy) ** 2) / (float(sxy[i]) ** 2) + ((z - float(sz[i])) ** 2) / (float(szz[i]) ** 2)))
        c = max(0.0, c + random.gauss(0.0, float(self.get_parameter("noise_std").value)))
        self.pub.publish(Float32(data=c))


def main():
    rclpy.init()
    node = GasSensorSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
