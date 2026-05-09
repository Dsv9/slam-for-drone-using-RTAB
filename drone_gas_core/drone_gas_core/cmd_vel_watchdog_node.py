from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CmdVelWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_watchdog_node")
        self.declare_parameter("input_cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("safe_cmd_vel_topic", "/drone/cmd_vel_safe")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("require_odom", True)
        self.declare_parameter("debug_print_cmd_vel", False)
        self.declare_parameter("debug_status_period_sec", 2.0)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.require_odom = bool(self.get_parameter("require_odom").value)
        self.last_cmd = Twist()
        self.last_cmd_t = None
        self.last_odom_t = None
        self._dbg_last = None
        self._dbg_last_state: Optional[Tuple[bool, bool, bool, bool]] = None
        self.create_subscription(
            Twist, self.get_parameter("input_cmd_vel_topic").value, self.cmd_cb, 20
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_cb, 20
        )
        self.pub = self.create_publisher(
            Twist, self.get_parameter("safe_cmd_vel_topic").value, 20
        )
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 1e-3), self.tick)

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_t = self.get_clock().now()

    def odom_cb(self, _msg: Odometry) -> None:
        self.last_odom_t = self.get_clock().now()

    def tick(self) -> None:
        now = self.get_clock().now()
        cmd_ok = (
            self.last_cmd_t is not None
            and (now - self.last_cmd_t).nanoseconds * 1e-9 <= self.cmd_timeout
        )
        odom_ok = (
            self.last_odom_t is not None
            and (now - self.last_odom_t).nanoseconds * 1e-9 <= self.odom_timeout
        )
        allow = cmd_ok and (odom_ok or not self.require_odom)
        self.pub.publish(self.last_cmd if allow else Twist())

        if bool(self.get_parameter("debug_print_cmd_vel").value):
            period = max(float(self.get_parameter("debug_status_period_sec").value), 0.5)
            state = (allow, cmd_ok, odom_ok, self.require_odom)
            tns = now.nanoseconds
            if self._dbg_last_state != state:
                self._dbg_last_state = state
                self.get_logger().info(
                    "cmd_vel watchdog: allow=%s cmd_fresh=%s odom_fresh=%s require_odom=%s"
                    % (allow, cmd_ok, odom_ok, self.require_odom)
                )
                self._dbg_last = tns
            elif (
                self._dbg_last is None
                or (tns - self._dbg_last) * 1e-9 >= period
            ):
                self._dbg_last = tns
                self.get_logger().info(
                    "cmd_vel watchdog: allow=%s cmd_fresh=%s odom_fresh=%s"
                    % (allow, cmd_ok, odom_ok)
                )


def main() -> None:
    rclpy.init()
    node = CmdVelWatchdogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
