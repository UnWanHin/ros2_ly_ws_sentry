#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


def clamp_rotate_level(value: int) -> int:
    return max(0, min(3, int(value)))


class ChassisSpinNode(Node):
    def __init__(self, rotate_level: int, hz: float, topic: str) -> None:
        super().__init__("chassis_spin_test")
        self.rotate_level = clamp_rotate_level(rotate_level)
        self.topic = topic
        self.pub = self.create_publisher(UInt8, topic, 10)
        self.timer = self.create_timer(1.0 / max(1.0, float(hz)), self._on_timer)
        self.frame = self._encode_firecode(self.rotate_level)
        self.get_logger().info(
            f"spin test started: rotate={self.rotate_level}, frame=0x{self.frame:02X}, topic={self.topic}"
        )

    @staticmethod
    def _encode_firecode(rotate_level: int) -> int:
        # FireCode bit layout:
        # bit0-1 FireStatus, bit2-3 CapState, bit4 HoleMode, bit5 AimMode, bit6-7 Rotate
        # Keep FireStatus/other bits at 0, only set rotate bits.
        return (clamp_rotate_level(rotate_level) & 0x03) << 6

    def _on_timer(self) -> None:
        msg = UInt8()
        msg.data = int(self.frame)
        self.pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="External chassis spin publisher via firecode.rotate")
    parser.add_argument("--rotate-level", type=int, default=1, help="Rotate level [0-3], default 1")
    parser.add_argument("--hz", type=float, default=20.0, help="Publish rate, default 20Hz")
    parser.add_argument("--topic", type=str, default="/ly/control/firecode", help="Firecode topic")
    args = parser.parse_args()

    rclpy.init()
    node = ChassisSpinNode(args.rotate_level, args.hz, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("chassis_spin_test interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

