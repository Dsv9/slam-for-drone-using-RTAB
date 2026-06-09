"""Publish odom -> base_link TF from /odom (same stamp as message).

Ensures RViz can transform sensor data into odom when VO publishes /odom but TF
is missing or stamped inconsistently.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf2_ros


def _valid_odom(msg: Odometry) -> bool:
    q = msg.pose.pose.orientation
    if abs(q.w) < 1e-6 and abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6:
        return False
    p = msg.pose.pose.position
    return math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)


class OdomTfBroadcasterNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster_node")
        self.declare_parameter("odom_topic", "/odom")
        self._br = tf2_ros.TransformBroadcaster(self)
        topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(Odometry, topic, self._odom_cb, 50)

    def _odom_cb(self, msg: Odometry) -> None:
        if not _valid_odom(msg):
            return
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or "odom"
        t.child_frame_id = msg.child_frame_id or "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
