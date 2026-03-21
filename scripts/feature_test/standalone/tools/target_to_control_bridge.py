#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from auto_aim_common.msg import Target
from gimbal_driver.msg import GimbalAngles
from std_msgs.msg import UInt8


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value}")


@dataclass
class TargetState:
    yaw: float
    pitch: float
    status: bool
    rx_ns: int


def encode_firecode(fire_status: int, aim_mode: bool) -> int:
    # bit0-1 FireStatus(00/11), bit2-3 CapState, bit4 HoleMode, bit5 AimMode, bit6-7 Rotate
    value = (fire_status & 0x03)
    if aim_mode:
        value |= (1 << 5)
    return value & 0xFF


class TargetToControlBridge(Node):
    def __init__(
        self,
        target_topic: str,
        angles_topic: str,
        firecode_topic: str,
        timeout_sec: float,
        publish_hz: float,
        enable_fire: bool,
        fire_hz: float,
        safe_firecode: int,
    ) -> None:
        super().__init__("standalone_target_to_control_bridge")
        self.timeout_ns = int(max(0.05, timeout_sec) * 1e9)
        self.enable_fire = bool(enable_fire)
        self.fire_interval_ns = int(1e9 / max(1.0, fire_hz))
        self.safe_firecode = max(0, min(255, int(safe_firecode)))

        self.pub_angles = self.create_publisher(GimbalAngles, angles_topic, 10)
        self.pub_firecode = self.create_publisher(UInt8, firecode_topic, 10)
        self.sub_target = self.create_subscription(Target, target_topic, self._on_target, 10)

        self.latest_target: Optional[TargetState] = None
        self.last_fire_toggle_ns = 0
        self.fire_status = 0
        self.last_aim_active = False

        timer_period = 1.0 / max(1.0, publish_hz)
        self.timer = self.create_timer(timer_period, self._on_timer)
        self.get_logger().info(
            "target bridge started: "
            f"target={target_topic} angles={angles_topic} firecode={firecode_topic} "
            f"enable_fire={self.enable_fire} publish_hz={publish_hz:.1f}"
        )

    def _on_target(self, msg: Target) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.latest_target = TargetState(
            yaw=float(msg.yaw),
            pitch=float(msg.pitch),
            status=bool(msg.status),
            rx_ns=now_ns,
        )

    def _publish_firecode(self, value: int) -> None:
        msg = UInt8()
        msg.data = int(value) & 0xFF
        self.pub_firecode.publish(msg)

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.latest_target is None or (now_ns - self.latest_target.rx_ns) > self.timeout_ns:
            if self.last_aim_active:
                self.get_logger().warn("target timeout, fallback to safe firecode")
            self.last_aim_active = False
            self.fire_status = 0
            self._publish_firecode(self.safe_firecode)
            return

        angles = GimbalAngles()
        angles.header.stamp = self.get_clock().now().to_msg()
        angles.yaw = float(self.latest_target.yaw)
        angles.pitch = float(self.latest_target.pitch)
        self.pub_angles.publish(angles)

        aim_active = True
        should_fire = self.enable_fire and self.latest_target.status
        if should_fire and (now_ns - self.last_fire_toggle_ns) >= self.fire_interval_ns:
            self.fire_status = 0b11 if self.fire_status == 0 else 0
            self.last_fire_toggle_ns = now_ns
        if not should_fire:
            self.fire_status = 0

        self._publish_firecode(encode_firecode(self.fire_status, aim_active))
        self.last_aim_active = aim_active


def main() -> None:
    parser = argparse.ArgumentParser(description="Bridge /ly/*/target to /ly/control/*")
    parser.add_argument("--target-topic", type=str, required=True)
    parser.add_argument("--angles-topic", type=str, default="/ly/control/angles")
    parser.add_argument("--firecode-topic", type=str, default="/ly/control/firecode")
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument("--publish-hz", type=float, default=50.0)
    parser.add_argument("--enable-fire", type=parse_bool, default=False)
    parser.add_argument("--fire-hz", type=float, default=20.0)
    parser.add_argument("--safe-firecode", type=int, default=0)
    args = parser.parse_args()

    rclpy.init()
    node = TargetToControlBridge(
        target_topic=args.target_topic,
        angles_topic=args.angles_topic,
        firecode_topic=args.firecode_topic,
        timeout_sec=args.timeout_sec,
        publish_hz=args.publish_hz,
        enable_fire=args.enable_fire,
        fire_hz=args.fire_hz,
        safe_firecode=args.safe_firecode,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("target bridge interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
