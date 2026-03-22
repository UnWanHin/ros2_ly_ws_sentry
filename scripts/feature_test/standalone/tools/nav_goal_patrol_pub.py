#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
import json
import random
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


def clamp_uint8(value: int) -> int:
    return max(0, min(255, int(value)))


def normalize_mode(value: str) -> str:
    normalized = str(value).strip().lower()
    if normalized in {"random", "rand"}:
        return "random"
    return "sequence"


@dataclass
class PatrolPlan:
    name: str
    goals: list[int]
    hold_sec: float
    speed_level: int
    mode: str
    disable_team_offset: bool


def load_patrol_plan(plan_file: str, plan_name: str | None) -> PatrolPlan:
    plan_path = Path(plan_file).expanduser().resolve()
    root = json.loads(plan_path.read_text(encoding="utf-8"))

    selected_name = plan_name or root.get("ActivePlan", "")
    plan_body: dict

    if isinstance(root.get("Plans"), dict):
        plans = root["Plans"]
        if not selected_name:
            selected_name = next(iter(plans))
        if selected_name not in plans:
            fallback_name = next(iter(plans))
            print(
                f"[WARN] plan '{selected_name}' not found in {plan_path}, fallback to '{fallback_name}'",
                flush=True,
            )
            selected_name = fallback_name
        plan_body = plans[selected_name]
    else:
        selected_name = selected_name or "<root>"
        plan_body = root

    goals = [clamp_uint8(goal) for goal in plan_body.get("Goals", [])]
    if not goals:
        raise ValueError(f"plan '{selected_name}' in {plan_path} has no Goals")

    return PatrolPlan(
        name=selected_name,
        goals=goals,
        hold_sec=max(1.0, float(plan_body.get("GoalHoldSec", 5))),
        speed_level=clamp_uint8(int(plan_body.get("SpeedLevel", 1))),
        mode=normalize_mode(plan_body.get("Mode", "sequence" if not plan_body.get("Random", False) else "random")),
        disable_team_offset=bool(plan_body.get("DisableTeamOffset", False)),
    )


class NaviGoalPatrolNode(Node):
    def __init__(self, plan: PatrolPlan, goal_topic: str, speed_topic: str, team: str, hz: float) -> None:
        super().__init__("navi_goal_patrol_pub")
        self.goal_pub = self.create_publisher(UInt8, goal_topic, 10)
        self.speed_pub = self.create_publisher(UInt8, speed_topic, 10)
        self.plan = plan
        self.team = team
        self.publish_hz = max(1.0, float(hz))
        self.current_index: int | None = None
        self.current_goal: int | None = None
        self.next_switch_time = 0.0

        self.create_timer(1.0 / self.publish_hz, self._tick)
        self._select_next(initial=True)

        self.get_logger().info(
            "navi patrol ready: "
            f"plan={self.plan.name} "
            f"mode={self.plan.mode} "
            f"goals={','.join(str(goal) for goal in self.plan.goals)} "
            f"hold_sec={self.plan.hold_sec:.1f} "
            f"speed={self.plan.speed_level} "
            f"team={self.team} "
            f"disable_team_offset={1 if self.plan.disable_team_offset else 0}"
        )

    def _resolve_goal(self, base_goal: int) -> int:
        if self.plan.disable_team_offset:
            return base_goal
        if self.team == "blue":
            return clamp_uint8(base_goal + 20)
        return base_goal

    def _next_index(self, initial: bool) -> int:
        if len(self.plan.goals) == 1:
            return 0
        if self.plan.mode != "random":
            return 0 if initial or self.current_index is None else (self.current_index + 1) % len(self.plan.goals)

        if initial or self.current_index is None:
            return random.randrange(len(self.plan.goals))

        next_index = self.current_index
        while next_index == self.current_index:
            next_index = random.randrange(len(self.plan.goals))
        return next_index

    def _select_next(self, initial: bool) -> None:
        self.current_index = self._next_index(initial=initial)
        base_goal = self.plan.goals[self.current_index]
        self.current_goal = self._resolve_goal(base_goal)
        self.next_switch_time = time.monotonic() + self.plan.hold_sec
        self.get_logger().info(
            f"switch goal: base_goal={base_goal} "
            f"effective_goal={self.current_goal} "
            f"next_in={self.plan.hold_sec:.1f}s"
        )

    def _publish_once(self) -> None:
        if self.current_goal is None:
            return

        goal_msg = UInt8()
        goal_msg.data = int(self.current_goal)
        self.goal_pub.publish(goal_msg)

        speed_msg = UInt8()
        speed_msg.data = int(self.plan.speed_level)
        self.speed_pub.publish(speed_msg)

    def _tick(self) -> None:
        if self.current_goal is None or time.monotonic() >= self.next_switch_time:
            self._select_next(initial=self.current_goal is None)
        self._publish_once()


def main() -> None:
    parser = argparse.ArgumentParser(description="Automatic patrol publisher for /ly/navi/goal")
    parser.add_argument("--plan-file", required=True, help="JSON plan file path")
    parser.add_argument("--plan", default="", help="Plan name override")
    parser.add_argument("--team", choices=["red", "blue"], default="red")
    parser.add_argument("--goal-topic", default="/ly/navi/goal")
    parser.add_argument("--speed-topic", default="/ly/navi/speed_level")
    parser.add_argument("--hz", type=float, default=2.0, help="Periodic publish rate")
    args = parser.parse_args()

    plan = load_patrol_plan(args.plan_file, args.plan or None)

    rclpy.init()
    node = NaviGoalPatrolNode(
        plan=plan,
        goal_topic=args.goal_topic,
        speed_topic=args.speed_topic,
        team=args.team,
        hz=args.hz,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
