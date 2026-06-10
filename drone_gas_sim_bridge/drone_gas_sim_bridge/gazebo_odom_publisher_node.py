"""Publish /odom from Gazebo model pose (ground truth for sim demo).

Reads bridged pose from Gazebo PosePublisher plugin (gz_ros_bridge.yaml).
Visual odometry can stay at 0,0,0 while VelocityControl moves the model.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class GazeboOdomPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_odom_publisher_node")
        self.declare_parameter("pose_topic", "/gazebo/simple_drone/pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        pose_topic = str(self.get_parameter("pose_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._child_frame = str(self.get_parameter("child_frame_id").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Odometry, odom_topic, 10)
        self._sub = self.create_subscription(Pose, pose_topic, self._on_pose, qos)
        self._prev: Optional[Tuple[float, float, float, float]] = None
        self._prev_t: Optional[float] = None
        self._got_pose = False
        self._warned = False

        self.create_timer(5.0, self._check_pose)
        self.get_logger().info(
            "Publishing %s from Gazebo pose topic %s (frame=%s child=%s)"
            % (odom_topic, pose_topic, self._odom_frame, self._child_frame)
        )

    def _check_pose(self) -> None:
        if self._got_pose:
            return
        if not self._warned:
            self._warned = True
            self.get_logger().warn(
                "No pose on %s yet — check PosePublisher plugin and gz_ros_bridge.yaml"
                % self.get_parameter("pose_topic").value
            )

    def _on_pose(self, msg: Pose) -> None:
        self._got_pose = True
        self._warned = False

        now = self.get_clock().now()
        t_sec = now.nanoseconds * 1e-9
        p = msg.position
        q = msg.orientation

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._child_frame
        odom.pose.pose.position.x = float(p.x)
        odom.pose.pose.position.y = float(p.y)
        odom.pose.pose.position.z = float(p.z)
        odom.pose.pose.orientation = q
        for i in (0, 7, 14):
            odom.pose.covariance[i] = 0.001

        if self._prev is not None and self._prev_t is not None:
            dt = max(t_sec - self._prev_t, 1e-3)
            px, py, pz, _ = self._prev
            odom.twist.twist.linear.x = (float(p.x) - px) / dt
            odom.twist.twist.linear.y = (float(p.y) - py) / dt
            odom.twist.twist.linear.z = (float(p.z) - pz) / dt
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            _, _, _, pyaw = self._prev
            dyaw = yaw - pyaw
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            odom.twist.twist.angular.z = dyaw / dt

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._prev = (float(p.x), float(p.y), float(p.z), yaw)
        self._prev_t = t_sec
        self._pub.publish(odom)


def main() -> None:
    rclpy.init()
    node = GazeboOdomPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
