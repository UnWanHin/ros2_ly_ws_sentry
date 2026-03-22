#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from gimbal_driver.msg import Vel
from std_msgs.msg import UInt8


def clamp_int8(value: int) -> int:
    return max(-128, min(127, int(value)))


def clamp_rotate_level(value: int) -> int:
    return max(0, min(3, int(value)))


@dataclass(frozen=True)
class TranslateProfile:
    speed_x: int
    speed_y: int
    period_sec: float


class ChassisSpinTranslateNode(Node):
    def __init__(
        self,
        profile: TranslateProfile,
        rotate_level: int,
        hz: float,
        vel_topic: str,
        firecode_topic: str,
        stop_repeat: int,
    ) -> None:
        super().__init__("chassis_spin_translate_test")
        self.profile = profile
        self.rotate_level = clamp_rotate_level(rotate_level)
        self.hz = max(1.0, float(hz))
        self.stop_repeat = max(1, int(stop_repeat))
        self.half_period_ns = max(1, int(self.profile.period_sec * 1e9 / 2.0))
        self.start_ns = self.get_clock().now().nanoseconds
        self.last_direction: int | None = None
        self.firecode_frame = self._encode_firecode(self.rotate_level)

        self.vel_pub = self.create_publisher(Vel, vel_topic, 10)
        self.firecode_pub = self.create_publisher(UInt8, firecode_topic, 10)
        self.timer = self.create_timer(1.0 / self.hz, self._on_timer)

        self.get_logger().info(
            "spin translate started: "
            f"rotate={self.rotate_level} "
            f"speed=({self.profile.speed_x},{self.profile.speed_y}) "
            f"period={self.profile.period_sec:.2f}s hz={self.hz:.1f} "
            f"vel_topic={vel_topic} firecode_topic={firecode_topic}"
        )

    @staticmethod
    def _encode_firecode(rotate_level: int) -> int:
        return (clamp_rotate_level(rotate_level) & 0x03) << 6

    def _current_direction(self) -> int:
        elapsed_ns = self.get_clock().now().nanoseconds - self.start_ns
        phase = (elapsed_ns // self.half_period_ns) % 2
        return 1 if phase == 0 else -1

    def _publish(self, speed_x: int, speed_y: int, firecode: int) -> None:
        vel_msg = Vel()
        vel_msg.x = int(speed_x)
        vel_msg.y = int(speed_y)
        self.vel_pub.publish(vel_msg)

        fire_msg = UInt8()
        fire_msg.data = int(firecode) & 0xFF
        self.firecode_pub.publish(fire_msg)

    def _publish_stop(self) -> None:
        for _ in range(self.stop_repeat):
            self._publish(0, 0, 0)

    def _on_timer(self) -> None:
        direction = self._current_direction()
        if direction != self.last_direction:
            leg = "forward" if direction > 0 else "backward"
            self.get_logger().info(
                f"switch leg={leg} vel=({direction * self.profile.speed_x},{direction * self.profile.speed_y})"
            )
            self.last_direction = direction

        self._publish(
            direction * self.profile.speed_x,
            direction * self.profile.speed_y,
            self.firecode_frame,
        )

    def shutdown(self) -> None:
        if not rclpy.ok():
            return
        try:
            self.get_logger().info("publishing stop frame before exit")
            self._publish_stop()
        except Exception:
            # External shutdown may invalidate the ROS context before cleanup runs.
            pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish /ly/control/vel back-and-forth with concurrent firecode.rotate"
    )
    parser.add_argument("--rotate-level", type=int, default=1, help="Rotate level [0-3], default 1")
    parser.add_argument("--speed-x", type=int, default=20, help="Velocity X amplitude, default 20")
    parser.add_argument("--speed-y", type=int, default=0, help="Velocity Y amplitude, default 0")
    parser.add_argument("--period-sec", type=float, default=4.0, help="Round-trip period, default 4s")
    parser.add_argument("--hz", type=float, default=20.0, help="Publish rate, default 20Hz")
    parser.add_argument("--vel-topic", type=str, default="/ly/control/vel", help="Velocity topic")
    parser.add_argument(
        "--firecode-topic",
        type=str,
        default="/ly/control/firecode",
        help="Firecode topic",
    )
    parser.add_argument(
        "--stop-repeat",
        type=int,
        default=5,
        help="How many zero frames to publish on shutdown, default 5",
    )
    args = parser.parse_args()

    profile = TranslateProfile(
        speed_x=clamp_int8(args.speed_x),
        speed_y=clamp_int8(args.speed_y),
        period_sec=max(0.2, float(args.period_sec)),
    )

    rclpy.init()
    node = ChassisSpinTranslateNode(
        profile=profile,
        rotate_level=args.rotate_level,
        hz=args.hz,
        vel_topic=args.vel_topic,
        firecode_topic=args.firecode_topic,
        stop_repeat=args.stop_repeat,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("chassis_spin_translate_test interrupted")
    except ExternalShutdownException:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
