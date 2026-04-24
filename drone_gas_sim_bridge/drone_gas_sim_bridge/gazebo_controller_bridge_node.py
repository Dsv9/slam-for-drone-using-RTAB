import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class GazeboControllerBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_controller_bridge_node")
        self.declare_parameter("input_cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("command_mode", "velocity")
        self.declare_parameter("gazebo_twist_topic", "/cmd_vel")
        self.declare_parameter("gazebo_pose_topic", "/drone/target_pose")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("max_linear_xy", 1.0)
        self.declare_parameter("max_linear_z", 0.8)
        self.declare_parameter("max_angular_z", 1.2)
        self.declare_parameter("pose_step_dt", 0.1)
        self.declare_parameter("pose_frame_id", "map")

        self.mode = self.get_parameter("command_mode").value
        self.pose_step_dt = float(self.get_parameter("pose_step_dt").value)
        self.max_linear_xy = float(self.get_parameter("max_linear_xy").value)
        self.max_linear_z = float(self.get_parameter("max_linear_z").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.pose_frame_id = self.get_parameter("pose_frame_id").value
        self.latest_cmd: Optional[Twist] = None
        self.latest_pose: Optional[Tuple[float, float, float, float]] = None

        self.create_subscription(Twist, self.get_parameter("input_cmd_vel_topic").value, self.cmd_cb, 20)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_cb, 20)
        self.twist_pub = self.create_publisher(Twist, self.get_parameter("gazebo_twist_topic").value, 20)
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter("gazebo_pose_topic").value, 20)
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 1e-3), self.on_timer)

    def clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def cmd_cb(self, msg: Twist) -> None:
        c = Twist()
        c.linear.x = self.clamp(msg.linear.x, -self.max_linear_xy, self.max_linear_xy)
        c.linear.y = self.clamp(msg.linear.y, -self.max_linear_xy, self.max_linear_xy)
        c.linear.z = self.clamp(msg.linear.z, -self.max_linear_z, self.max_linear_z)
        c.angular.z = self.clamp(msg.angular.z, -self.max_angular_z, self.max_angular_z)
        self.latest_cmd = c

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_pose = (p.x, p.y, p.z, yaw)

    def on_timer(self) -> None:
        if self.latest_cmd is None:
            return
        if self.mode == "pose":
            self.publish_pose_step(self.latest_cmd)
        else:
            self.twist_pub.publish(self.latest_cmd)

    def publish_pose_step(self, cmd: Twist) -> None:
        if self.latest_pose is None:
            self.twist_pub.publish(cmd)
            return
        x, y, z, yaw = self.latest_pose
        dt = max(self.pose_step_dt, 1e-3)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.pose_frame_id
        pose.pose.position.x = x + cmd.linear.x * dt
        pose.pose.position.y = y + cmd.linear.y * dt
        pose.pose.position.z = z + cmd.linear.z * dt
        yaw_next = yaw + cmd.angular.z * dt
        pose.pose.orientation.z = math.sin(yaw_next * 0.5)
        pose.pose.orientation.w = math.cos(yaw_next * 0.5)
        self.pose_pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = GazeboControllerBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
