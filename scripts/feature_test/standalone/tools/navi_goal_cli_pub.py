#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt8


def clamp_uint8(value: int) -> int:
    return max(0, min(255, int(value)))


class NaviGoalCliNode(Node):
    def __init__(self, goal_topic: str, speed_topic: str, hz: float, init_speed: int) -> None:
        super().__init__("navi_goal_cli_pub")
        self.goal_pub = self.create_publisher(UInt8, goal_topic, 10)
        self.speed_pub = self.create_publisher(UInt8, speed_topic, 10)
        self.goal_topic = goal_topic
        self.speed_topic = speed_topic
        self.publish_hz = max(1.0, float(hz))

        self._goal_lock = threading.Lock()
        self._current_goal: int | None = None
        self._current_speed: int = clamp_uint8(init_speed)

        self.create_timer(1.0 / self.publish_hz, self._publish_periodic)

        self.get_logger().info(
            f"navi goal cli ready: goal_topic={goal_topic}, speed_topic={speed_topic}, hz={self.publish_hz:.1f}"
        )

    def set_goal(self, goal: int) -> None:
        goal = clamp_uint8(goal)
        with self._goal_lock:
            self._current_goal = goal
        self.publish_once()
        self.get_logger().info(f"set goal={goal}")

    def set_speed(self, speed: int) -> None:
        speed = clamp_uint8(speed)
        with self._goal_lock:
            self._current_speed = speed
        self.publish_once()
        self.get_logger().info(f"set speed={speed}")

    def get_state(self) -> tuple[int | None, int]:
        with self._goal_lock:
            return self._current_goal, self._current_speed

    def publish_once(self) -> None:
        goal, speed = self.get_state()
        if goal is None:
            return
        goal_msg = UInt8()
        goal_msg.data = int(goal)
        self.goal_pub.publish(goal_msg)

        speed_msg = UInt8()
        speed_msg.data = int(speed)
        self.speed_pub.publish(speed_msg)

    def _publish_periodic(self) -> None:
        self.publish_once()


def print_help_text() -> None:
    print("")
    print("Input command:")
    print("  <id>          -> set /ly/navi/goal (0~255)")
    print("  s <speed>     -> set /ly/navi/speed_level (0~255)")
    print("  <id> <speed>  -> set both goal and speed")
    print("  p             -> print current state")
    print("  q             -> quit")
    print("")


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive publisher for /ly/navi/goal")
    parser.add_argument("--goal-topic", type=str, default="/ly/navi/goal")
    parser.add_argument("--speed-topic", type=str, default="/ly/navi/speed_level")
    parser.add_argument("--hz", type=float, default=2.0, help="Periodic publish rate")
    parser.add_argument("--init-speed", type=int, default=1, help="Initial speed_level")
    args = parser.parse_args()

    rclpy.init()
    node = NaviGoalCliNode(
        goal_topic=args.goal_topic,
        speed_topic=args.speed_topic,
        hz=args.hz,
        init_speed=args.init_speed,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print_help_text()
    try:
        while rclpy.ok():
            raw = input("navi> ").strip()
            if not raw:
                continue
            if raw.lower() in {"q", "quit", "exit"}:
                break
            if raw.lower() in {"h", "help"}:
                print_help_text()
                continue
            if raw.lower() in {"p", "print", "status"}:
                goal, speed = node.get_state()
                print(f"current goal={goal}, speed={speed}")
                continue

            parts = raw.split()
            try:
                if len(parts) == 2 and parts[0].lower() == "s":
                    node.set_speed(int(parts[1]))
                    continue
                if len(parts) == 1:
                    node.set_goal(int(parts[0]))
                    continue
                if len(parts) == 2:
                    node.set_goal(int(parts[0]))
                    node.set_speed(int(parts[1]))
                    continue
            except ValueError:
                pass

            print("Invalid input. Type 'help' to show usage.")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        # Give spin thread a short moment to exit cleanly.
        time.sleep(0.1)


if __name__ == "__main__":
    main()
