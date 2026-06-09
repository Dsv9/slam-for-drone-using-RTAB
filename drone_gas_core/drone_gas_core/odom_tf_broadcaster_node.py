"""Broadcast odom -> base_link on /tf from /odom.

Republishes at publish_hz with current sim clock stamp so RViz can transform
PointCloud2 (rgbd_camera) into Fixed Frame odom. Message-stamp-only TF often fails
when sensor stamps and VO stamps differ slightly.
"""
from __future__ import annotations

import math
import time
from typing import Optional

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
    if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
        return False
    cov = msg.pose.covariance
    if len(cov) >= 36:
        diag = [cov[i] for i in (0, 7, 14, 21, 28, 35)]
        if all(v >= 9999.0 for v in diag) and abs(p.x) < 1e-6 and abs(p.y) < 1e-6:
            return False
    return True


class OdomTfBroadcasterNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster_node")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("debug_tf", True)

        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._br = tf2_ros.TransformBroadcaster(self)
        self._latest: Optional[TransformStamped] = None
        self._dbg_last = 0.0

        topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(Odometry, topic, self._odom_cb, 50)
        hz = max(float(self.get_parameter("publish_hz").value), 5.0)
        self.create_timer(1.0 / hz, self._publish_tf)

    def _odom_cb(self, msg: Odometry) -> None:
        if not _valid_odom(msg):
            return
        t = TransformStamped()
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = float(msg.pose.pose.position.x)
        t.transform.translation.y = float(msg.pose.pose.position.y)
        t.transform.translation.z = float(msg.pose.pose.position.z)
        t.transform.rotation = msg.pose.pose.orientation
        self._latest = t

    def _publish_tf(self) -> None:
        if self._latest is None:
            now = time.monotonic()
            if bool(self.get_parameter("debug_tf").value) and (now - self._dbg_last) >= 2.0:
                self._dbg_last = now
                self.get_logger().warn("Waiting for valid /odom before publishing odom->base_link TF")
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform = self._latest.transform
        self._br.sendTransform(t)

        now = time.monotonic()
        if bool(self.get_parameter("debug_tf").value) and (now - self._dbg_last) >= 5.0:
            self._dbg_last = now
            tr = t.transform.translation
            self.get_logger().info(
                "[tf] publishing %s -> %s x=%.3f y=%.3f z=%.3f"
                % (self._odom_frame, self._base_frame, tr.x, tr.y, tr.z)
            )


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
