#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse

import rclpy
from rclpy.node import Node

from gimbal_driver.msg import GimbalAngles
from std_msgs.msg import UInt8


class ScanGimbalNode(Node):
    def __init__(
        self,
        yaw_min: float,
        yaw_max: float,
        pitch: float,
        step_deg: float,
        hz: float,
        angles_topic: str,
        firecode_topic: str,
        safe_firecode: int,
    ) -> None:
        super().__init__("scan_gimbal_test")
        if yaw_min > yaw_max:
            yaw_min, yaw_max = yaw_max, yaw_min

        self.yaw_min = yaw_min
        self.yaw_max = yaw_max
        self.pitch = pitch
        self.step_deg = abs(step_deg) if abs(step_deg) > 0.01 else 0.5
        self.yaw = yaw_min
        self.direction = 1.0
        self.safe_firecode = max(0, min(255, safe_firecode))

        self.angles_pub = self.create_publisher(GimbalAngles, angles_topic, 10)
        self.firecode_pub = self.create_publisher(UInt8, firecode_topic, 10)
        period = 1.0 / max(1.0, hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"scan started: yaw=[{self.yaw_min:.2f},{self.yaw_max:.2f}] "
            f"pitch={self.pitch:.2f} step={self.step_deg:.2f} "
            f"angles_topic={angles_topic} firecode_topic={firecode_topic}"
        )

    def _on_timer(self) -> None:
        msg = GimbalAngles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.yaw = float(self.yaw)
        msg.pitch = float(self.pitch)
        self.angles_pub.publish(msg)

        fire_msg = UInt8()
        fire_msg.data = self.safe_firecode
        self.firecode_pub.publish(fire_msg)

        self.yaw += self.direction * self.step_deg
        if self.yaw >= self.yaw_max:
            self.yaw = self.yaw_max
            self.direction = -1.0
        elif self.yaw <= self.yaw_min:
            self.yaw = self.yaw_min
            self.direction = 1.0


def main() -> None:
    parser = argparse.ArgumentParser(description="External gimbal scan publisher")
    parser.add_argument("--yaw-min", type=float, default=-15.0)
    parser.add_argument("--yaw-max", type=float, default=15.0)
    parser.add_argument("--pitch", type=float, default=8.0)
    parser.add_argument("--step-deg", type=float, default=1.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--angles-topic", type=str, default="/ly/control/angles")
    parser.add_argument("--firecode-topic", type=str, default="/ly/control/firecode")
    parser.add_argument("--safe-firecode", type=int, default=0)
    cli_args = parser.parse_args()

    rclpy.init()
    node = ScanGimbalNode(
        yaw_min=cli_args.yaw_min,
        yaw_max=cli_args.yaw_max,
        pitch=cli_args.pitch,
        step_deg=cli_args.step_deg,
        hz=cli_args.hz,
        angles_topic=cli_args.angles_topic,
        firecode_topic=cli_args.firecode_topic,
        safe_firecode=cli_args.safe_firecode,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("scan_gimbal_test interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
