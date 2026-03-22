#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from gimbal_driver.msg import Vel


def clamp_int8(value: int) -> int:
    return max(-128, min(127, int(value)))


@dataclass
class VelocityProfile:
    name: str
    speed_x: int
    speed_y: int
    period_sec: float


class ChassisVelNode(Node):
    def __init__(self, profile: VelocityProfile, hz: float, topic: str) -> None:
        super().__init__("chassis_vel_test")
        self.profile = profile
        self.publisher = self.create_publisher(Vel, topic, 10)
        self.tick = 0
        self.hz = max(1.0, hz)
        self.timer = self.create_timer(1.0 / self.hz, self._on_timer)
        self.get_logger().info(
            f"vel test started: profile={self.profile.name} "
            f"x={self.profile.speed_x} y={self.profile.speed_y} "
            f"period={self.profile.period_sec:.2f}s topic={topic}"
        )

    def _resolve_speed(self) -> tuple[int, int]:
        if self.profile.name != "square":
            return self.profile.speed_x, self.profile.speed_y

        total_ticks = max(4, int(self.profile.period_sec * self.hz))
        phase_len = max(1, total_ticks // 4)
        phase = (self.tick // phase_len) % 4
        if phase == 0:
            return self.profile.speed_x, 0
        if phase == 1:
            return 0, self.profile.speed_y
        if phase == 2:
            return -self.profile.speed_x, 0
        return 0, -self.profile.speed_y

    def _on_timer(self) -> None:
        x, y = self._resolve_speed()
        msg = Vel()
        msg.x = int(x)
        msg.y = int(y)
        self.publisher.publish(msg)
        self.tick += 1


def main() -> None:
    parser = argparse.ArgumentParser(description="External chassis velocity publisher")
    parser.add_argument("--profile", type=str, default="constant", choices=["constant", "square"])
    parser.add_argument("--speed-x", type=int, default=20)
    parser.add_argument("--speed-y", type=int, default=0)
    parser.add_argument("--period-sec", type=float, default=8.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--topic", type=str, default="/ly/control/vel")
    cli_args = parser.parse_args()

    profile = VelocityProfile(
        name=cli_args.profile,
        speed_x=clamp_int8(cli_args.speed_x),
        speed_y=clamp_int8(cli_args.speed_y),
        period_sec=max(1.0, float(cli_args.period_sec)),
    )

    rclpy.init()
    node = ChassisVelNode(profile=profile, hz=cli_args.hz, topic=cli_args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("chassis_vel_test interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
