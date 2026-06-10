"""Publish /odom from Gazebo model pose (ground truth for sim demo).

Visual odometry can stay at 0,0,0 while VelocityControl moves the model.
This node reads the real Gazebo pose so gas mapping and TF track movement.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.srv import GetEntityState


class GazeboOdomPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_odom_publisher_node")
        self.declare_parameter("world_name", "default")
        self.declare_parameter("model_name", "simple_drone")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("reference_frame", "world")

        self._world = str(self.get_parameter("world_name").value)
        self._model = str(self.get_parameter("model_name").value)
        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._child_frame = str(self.get_parameter("child_frame_id").value)
        self._ref = str(self.get_parameter("reference_frame").value)

        self._pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 20
        )
        self._client = self.create_client(
            GetEntityState, f"/world/{self._world}/get_entity_state"
        )
        self._prev: Optional[Tuple[float, float, float, float]] = None
        self._prev_t: Optional[float] = None
        self._warned = False

        hz = max(float(self.get_parameter("publish_hz").value), 5.0)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            "Gazebo ground-truth /odom from world=%s model=%s"
            % (self._world, self._model)
        )

    def _tick(self) -> None:
        if not self._client.service_is_ready():
            if not self._warned:
                self._warned = True
                self.get_logger().warn("Waiting for Gazebo get_entity_state service...")
            return
        self._warned = False
        req = GetEntityState.Request()
        req.name = self._model
        req.reference_frame = self._ref
        future = self._client.call_async(req)
        future.add_done_callback(self._on_state)

    def _on_state(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn("get_entity_state failed: %s" % exc)
            return
        if resp is None or not resp.success:
            return

        now = self.get_clock().now()
        t_sec = now.nanoseconds * 1e-9
        p = resp.state.pose.position
        q = resp.state.pose.orientation

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
