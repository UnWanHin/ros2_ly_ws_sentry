#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value}")


class ControlModePublisher(Node):
    def __init__(
        self,
        aa_enable: bool,
        ra_enable: bool,
        outpost_enable: bool,
        bt_target: int,
        hz: float,
    ) -> None:
        super().__init__("standalone_control_mode_publisher")
        self.aa_enable = bool(aa_enable)
        self.ra_enable = bool(ra_enable)
        self.outpost_enable = bool(outpost_enable)
        self.bt_target = max(0, min(255, int(bt_target)))

        self.pub_aa = self.create_publisher(Bool, "/ly/aa/enable", 10)
        self.pub_ra = self.create_publisher(Bool, "/ly/ra/enable", 10)
        self.pub_outpost = self.create_publisher(Bool, "/ly/outpost/enable", 10)
        self.pub_target = self.create_publisher(UInt8, "/ly/bt/target", 10)

        period = 1.0 / max(1.0, float(hz))
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "control mode publisher started: "
            f"aa_enable={self.aa_enable} ra_enable={self.ra_enable} "
            f"outpost_enable={self.outpost_enable} bt_target={self.bt_target} hz={1.0 / period:.1f}"
        )

    def _on_timer(self) -> None:
        aa_msg = Bool()
        aa_msg.data = self.aa_enable
        self.pub_aa.publish(aa_msg)

        ra_msg = Bool()
        ra_msg.data = self.ra_enable
        self.pub_ra.publish(ra_msg)

        outpost_msg = Bool()
        outpost_msg.data = self.outpost_enable
        self.pub_outpost.publish(outpost_msg)

        target_msg = UInt8()
        target_msg.data = self.bt_target
        self.pub_target.publish(target_msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish standalone mode switch topics")
    parser.add_argument("--aa-enable", type=parse_bool, default=True)
    parser.add_argument("--ra-enable", type=parse_bool, default=False)
    parser.add_argument("--outpost-enable", type=parse_bool, default=False)
    parser.add_argument("--bt-target", type=int, default=6)
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = ControlModePublisher(
        aa_enable=args.aa_enable,
        ra_enable=args.ra_enable,
        outpost_enable=args.outpost_enable,
        bt_target=args.bt_target,
        hz=args.hz,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("control mode publisher interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
