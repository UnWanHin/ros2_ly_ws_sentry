#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import re
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray


class ManualGoalInputNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "manual_goal_input_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.raw_topic = str(self.get_param_compat("raw_topic", "/ly/navi/goal_pos_raw"))
        self.goal_topic = str(self.get_param_compat("goal_topic", "/ly/navi/goal_pos"))
        self.input_unit = str(self.get_param_compat("input_unit", "cm")).strip().lower()
        if self.input_unit not in {"cm", "m"}:
            self.get_logger().warn(
                f"Unsupported input_unit={self.input_unit}, fallback to 'cm'."
            )
            self.input_unit = "cm"
        self.echo_goal = bool(self.get_param_compat("echo_goal", True))

        self.pub_raw = self.create_publisher(UInt16MultiArray, self.raw_topic, 10)
        self.sub_goal = self.create_subscription(
            UInt16MultiArray,
            self.goal_topic,
            self.on_goal_pos,
            10,
        )

        self.stop_event = threading.Event()
        self.seq = 0

        self.get_logger().info(
            f"manual goal input ready: raw_topic={self.raw_topic} "
            f"goal_topic={self.goal_topic} input_unit={self.input_unit}"
        )
        self.get_logger().info(
            "Input format: x y. Example: '1200 650' (cm) or '12.0 6.5' (m). Type 'q' to quit."
        )

        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def get_param_compat(self, name: str, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def to_cm(self, value_text: str) -> Optional[int]:
        try:
            number = float(value_text)
        except ValueError:
            return None
        if self.input_unit == "m":
            number *= 100.0
        return int(round(number))

    def on_goal_pos(self, msg: UInt16MultiArray) -> None:
        if not self.echo_goal:
            return
        if len(msg.data) < 2:
            return
        self.get_logger().info(
            f"goal_pos <= x={int(msg.data[0])} cm y={int(msg.data[1])} cm"
        )

    def input_loop(self) -> None:
        prompt = "[manual_goal_input] x y > "
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                line = input(prompt)
            except EOFError:
                self.get_logger().info("stdin closed, exit.")
                break
            except KeyboardInterrupt:
                break

            text = line.strip()
            if not text:
                continue
            if text.lower() in {"q", "quit", "exit"}:
                break

            parts = [p for p in re.split(r"[\s,]+", text) if p]
            if len(parts) < 2:
                self.get_logger().warn("Invalid input. Need two numbers: x y")
                continue

            x_cm = self.to_cm(parts[0])
            y_cm = self.to_cm(parts[1])
            if x_cm is None or y_cm is None:
                self.get_logger().warn("Invalid number. Example: 1200 650")
                continue
            if x_cm < 0 or y_cm < 0 or x_cm > 65535 or y_cm > 65535:
                self.get_logger().warn(
                    f"Out of range. Need 0~65535 cm, got x={x_cm} y={y_cm}"
                )
                continue

            msg = UInt16MultiArray()
            msg.data = [x_cm, y_cm]
            self.pub_raw.publish(msg)
            self.seq += 1
            self.get_logger().info(
                f"published raw_goal[{self.seq}] => x={x_cm} cm y={y_cm} cm "
                f"topic={self.raw_topic}"
            )

        self.stop_event.set()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualGoalInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
