#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from auto_aim_common.msg import Target
from gimbal_driver.msg import GimbalAngles, Vel
from std_msgs.msg import Bool, UInt8


def encode_firecode(fire_status: int, aim_mode: bool) -> int:
    # bit0-1 FireStatus, bit5 AimMode
    value = fire_status & 0x03
    if aim_mode:
        value |= (1 << 5)
    return value & 0xFF


@dataclass
class TargetState:
    yaw: float
    pitch: float
    status: bool
    rx_ns: int


class BuffTestBridge(Node):
    def __init__(self) -> None:
        super().__init__("buff_test_bridge")

        self.declare_parameter("target_topic", "/ly/buff/target")
        self.declare_parameter("aa_enable_topic", "/ly/aa/enable")
        self.declare_parameter("ra_enable_topic", "/ly/ra/enable")
        self.declare_parameter("control_angles_topic", "/ly/control/angles")
        self.declare_parameter("control_firecode_topic", "/ly/control/firecode")
        self.declare_parameter("control_vel_topic", "/ly/control/vel")
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("publish_hz", 80.0)
        self.declare_parameter("gate_hz", 2.0)
        self.declare_parameter("fire_hz", 20.0)
        self.declare_parameter("enable_fire", True)
        self.declare_parameter("zero_velocity", True)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.aa_enable_topic = str(self.get_parameter("aa_enable_topic").value)
        self.ra_enable_topic = str(self.get_parameter("ra_enable_topic").value)
        self.control_angles_topic = str(self.get_parameter("control_angles_topic").value)
        self.control_firecode_topic = str(self.get_parameter("control_firecode_topic").value)
        self.control_vel_topic = str(self.get_parameter("control_vel_topic").value)

        timeout_sec = float(self.get_parameter("timeout_sec").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        gate_hz = float(self.get_parameter("gate_hz").value)
        fire_hz = float(self.get_parameter("fire_hz").value)
        self.enable_fire = bool(self.get_parameter("enable_fire").value)
        self.zero_velocity = bool(self.get_parameter("zero_velocity").value)

        self.timeout_ns = int(max(timeout_sec, 0.05) * 1e9)
        self.fire_interval_ns = int(1e9 / max(fire_hz, 1.0))

        self.pub_aa_enable = self.create_publisher(Bool, self.aa_enable_topic, 10)
        self.pub_ra_enable = self.create_publisher(Bool, self.ra_enable_topic, 10)
        self.pub_angles = self.create_publisher(GimbalAngles, self.control_angles_topic, 50)
        self.pub_firecode = self.create_publisher(UInt8, self.control_firecode_topic, 50)
        self.pub_vel = self.create_publisher(Vel, self.control_vel_topic, 20)

        self.sub_target = self.create_subscription(Target, self.target_topic, self._on_target, 20)

        self.latest_target: Optional[TargetState] = None
        self.last_fire_toggle_ns = 0
        self.fire_status = 0
        self.last_timeout_log_ns = 0

        self.control_timer = self.create_timer(1.0 / max(publish_hz, 1.0), self._on_control_timer)
        self.gate_timer = self.create_timer(1.0 / max(gate_hz, 0.5), self._on_gate_timer)

        self._on_gate_timer()
        self.get_logger().info(
            "buff_test_bridge started: "
            f"target={self.target_topic} -> angles={self.control_angles_topic}, firecode={self.control_firecode_topic}, "
            f"aa=false@{self.aa_enable_topic}, ra=true@{self.ra_enable_topic}, "
            f"enable_fire={int(self.enable_fire)} fire_hz={fire_hz:.1f}"
        )

    def _on_target(self, msg: Target) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.latest_target = TargetState(
            yaw=float(msg.yaw),
            pitch=float(msg.pitch),
            status=bool(msg.status),
            rx_ns=now_ns,
        )

    def _on_gate_timer(self) -> None:
        aa_msg = Bool()
        aa_msg.data = False
        self.pub_aa_enable.publish(aa_msg)

        ra_msg = Bool()
        ra_msg.data = True
        self.pub_ra_enable.publish(ra_msg)

    def _publish_zero_velocity(self) -> None:
        if not self.zero_velocity:
            return
        vel_msg = Vel()
        vel_msg.x = 0
        vel_msg.y = 0
        self.pub_vel.publish(vel_msg)

    def _publish_firecode(self, value: int) -> None:
        msg = UInt8()
        msg.data = int(value) & 0xFF
        self.pub_firecode.publish(msg)

    def _on_control_timer(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        target_fresh = (
            self.latest_target is not None and
            (now_ns - self.latest_target.rx_ns) <= self.timeout_ns
        )

        if not target_fresh:
            self.fire_status = 0
            self._publish_firecode(encode_firecode(self.fire_status, False))
            self._publish_zero_velocity()
            if now_ns - self.last_timeout_log_ns > int(2e9):
                self.get_logger().warn("buff target timeout, publish safe firecode.")
                self.last_timeout_log_ns = now_ns
            return

        angles = GimbalAngles()
        angles.header.stamp = now.to_msg()
        angles.yaw = float(self.latest_target.yaw)
        angles.pitch = float(self.latest_target.pitch)
        self.pub_angles.publish(angles)

        should_fire = self.enable_fire and bool(self.latest_target.status)
        if should_fire and (now_ns - self.last_fire_toggle_ns) >= self.fire_interval_ns:
            self.fire_status = 0b11 if self.fire_status == 0 else 0
            self.last_fire_toggle_ns = now_ns
        if not should_fire:
            self.fire_status = 0

        self._publish_firecode(encode_firecode(self.fire_status, True))
        self._publish_zero_velocity()


def main() -> None:
    rclpy.init()
    node = BuffTestBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("buff_test_bridge interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
