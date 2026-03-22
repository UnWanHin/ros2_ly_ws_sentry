#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
import math
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
class SineTranslateProfile:
    speed_x: int
    speed_y_amplitude: int
    period_sec: float
    phase_rad: float


class ChassisSpinSineTranslateNode(Node):
    def __init__(
        self,
        profile: SineTranslateProfile,
        rotate_level: int,
        hz: float,
        vel_topic: str,
        firecode_topic: str,
        stop_repeat: int,
    ) -> None:
        super().__init__("chassis_spin_sine_translate_test")
        self.profile = profile
        self.rotate_level = clamp_rotate_level(rotate_level)
        self.hz = max(1.0, float(hz))
        self.stop_repeat = max(1, int(stop_repeat))
        self.angular_frequency = (2.0 * math.pi) / max(0.2, self.profile.period_sec)
        self.start_ns = self.get_clock().now().nanoseconds
        self.last_direction: int | None = None
        self.firecode_frame = self._encode_firecode(self.rotate_level)

        self.vel_pub = self.create_publisher(Vel, vel_topic, 10)
        self.firecode_pub = self.create_publisher(UInt8, firecode_topic, 10)
        self.timer = self.create_timer(1.0 / self.hz, self._on_timer)

        self.get_logger().info(
            "spin sine translate started: "
            f"rotate={self.rotate_level} "
            f"speed_x={self.profile.speed_x} "
            f"speed_y_amp={self.profile.speed_y_amplitude} "
            f"period={self.profile.period_sec:.2f}s "
            f"phase_deg={math.degrees(self.profile.phase_rad):.1f} "
            f"hz={self.hz:.1f} "
            f"vel_topic={vel_topic} firecode_topic={firecode_topic}"
        )

    @staticmethod
    def _encode_firecode(rotate_level: int) -> int:
        return (clamp_rotate_level(rotate_level) & 0x03) << 6

    def _elapsed_sec(self) -> float:
        return (self.get_clock().now().nanoseconds - self.start_ns) / 1e9

    def _current_speed_y(self) -> int:
        value = self.profile.speed_y_amplitude * math.sin(
            (self.angular_frequency * self._elapsed_sec()) + self.profile.phase_rad
        )
        return clamp_int8(round(value))

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
        speed_y = self._current_speed_y()
        direction = 0 if speed_y == 0 else (1 if speed_y > 0 else -1)

        if direction != self.last_direction and direction != 0:
            leg = "positive_y" if direction > 0 else "negative_y"
            self.get_logger().info(
                f"switch leg={leg} vel=({self.profile.speed_x},{speed_y})"
            )
            self.last_direction = direction

        self._publish(self.profile.speed_x, speed_y, self.firecode_frame)

    def shutdown(self) -> None:
        if not rclpy.ok():
            return
        try:
            self.get_logger().info("publishing stop frame before exit")
            self._publish_stop()
        except Exception:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Publish /ly/control/vel with y = A*sin(2*pi*t/T + phase) "
            "and concurrent firecode.rotate. Lower protocol rotate is 0..3 only."
        )
    )
    parser.add_argument("--rotate-level", type=int, default=1, help="Rotate level [0-3], default 1")
    parser.add_argument("--speed-x", type=int, default=0, help="Constant velocity X command, default 0")
    parser.add_argument(
        "--speed-y-amplitude",
        type=int,
        default=20,
        help="Velocity Y sine amplitude in /ly/control/vel int8 command units, default 20",
    )
    parser.add_argument("--period-sec", type=float, default=2.0, help="Sine period in seconds, default 2s")
    parser.add_argument("--phase-deg", type=float, default=0.0, help="Initial sine phase in degrees, default 0")
    parser.add_argument("--hz", type=float, default=30.0, help="Publish rate, default 30Hz")
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

    profile = SineTranslateProfile(
        speed_x=clamp_int8(args.speed_x),
        speed_y_amplitude=clamp_int8(args.speed_y_amplitude),
        period_sec=max(0.2, float(args.period_sec)),
        phase_rad=math.radians(float(args.phase_deg)),
    )

    rclpy.init()
    node = ChassisSpinSineTranslateNode(
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
            node.get_logger().info("chassis_spin_sine_translate_test interrupted")
    except ExternalShutdownException:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
