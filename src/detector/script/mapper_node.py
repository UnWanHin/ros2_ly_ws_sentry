#!/usr/bin/env python3
import argparse
import math
from typing import List, Optional, Set

import rclpy
from rclpy.node import Node

from auto_aim_common.msg import Armors, Target
from gimbal_driver.msg import GimbalAngles
from std_msgs.msg import Bool, UInt8


TARGET_ID_MIN = 0
TARGET_ID_MAX = 255


def str2bool(value: str) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in ("true", "1", "yes", "y", "on"):
        return True
    if lowered in ("false", "0", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def parse_target_id(value: str) -> int:
    parsed = int(value)
    if parsed < TARGET_ID_MIN or parsed > TARGET_ID_MAX:
        raise argparse.ArgumentTypeError(
            f"target id out of range [{TARGET_ID_MIN}, {TARGET_ID_MAX}]: {parsed}"
        )
    return parsed


def parse_target_priority(raw_priority: str, fallback_target_id: int) -> List[int]:
    if raw_priority is None:
        return [fallback_target_id]
    tokens = [token.strip() for token in raw_priority.split(",") if token.strip()]
    if not tokens:
        return [fallback_target_id]

    ordered_unique: List[int] = []
    seen: Set[int] = set()
    for token in tokens:
        target_id = parse_target_id(token)
        if target_id not in seen:
            seen.add(target_id)
            ordered_unique.append(target_id)
    return ordered_unique or [fallback_target_id]


class TargetToGimbalNode(Node):
    def __init__(
        self,
        is_red=True,
        target_id=6,
        target_priority: Optional[List[int]] = None,
        publish_team=False,
        publish_target=True,
        enable_fire=True,
        auto_fire=True,
        target_timeout_s=1.0,
        diag_period=1.0,
    ):
        super().__init__("target_to_gimbal_mapper")

        self.is_red = is_red
        self.publish_team = publish_team
        self.publish_target = publish_target
        self.enable_fire = enable_fire
        self.auto_fire = auto_fire
        self.target_timeout_s = max(0.2, float(target_timeout_s))
        self.diag_period = max(0.2, float(diag_period))

        if target_priority:
            self.target_priority = target_priority
        else:
            self.target_priority = [target_id]
        self.priority_rank = {
            candidate: index for index, candidate in enumerate(self.target_priority)
        }
        self.default_target_id = self.target_priority[0]
        self.selected_target_id = self.default_target_id
        self.current_detected_types: Set[int] = set()
        self.last_armors_time = self.get_clock().now()
        self.switch_confirm_frames = 3
        self.pending_target_id: Optional[int] = None
        self.pending_target_frames = 0

        self.fire_status = 0
        self.last_fire_command = False

        self.rx_target_count = 0
        self.rx_armors_count = 0
        self.tx_angle_count = 0
        self.tx_fire_count = 0
        self.tx_bt_target_count = 0
        self.tx_team_count = 0
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        self.last_status = False
        self.last_diag_time = self.get_clock().now()

        self.control_angles_topic = self.resolve_topic_name("/ly/control/angles")
        self.control_firecode_topic = self.resolve_topic_name("/ly/control/firecode")
        self.bt_target_topic = self.resolve_topic_name("/ly/bt/target")
        self.team_topic = self.resolve_topic_name("/ly/me/is_team_red")

        self.last_warned_angle_pubs = 0
        self.last_warned_fire_pubs = 0
        self.last_warned_bt_pubs = 0

        self.target_sub = self.create_subscription(
            Target,
            "/ly/predictor/target",
            self.target_callback,
            10,
        )
        self.armors_sub = self.create_subscription(
            Armors,
            "/ly/detector/armors",
            self.armors_callback,
            10,
        )
        self.gimbal_pub = self.create_publisher(GimbalAngles, "/ly/control/angles", 10)
        self.firecode_pub = self.create_publisher(UInt8, "/ly/control/firecode", 10)
        self.team_pub = self.create_publisher(Bool, "/ly/me/is_team_red", 10) if publish_team else None
        self.bt_target_pub = self.create_publisher(UInt8, "/ly/bt/target", 10) if publish_target else None

        self.timer = self.create_timer(0.1, self.timer_callback)

        if self.enable_fire:
            self.publish_firecode(self.fire_status)
            self.get_logger().info(f"初始火控状态: 0b{self.fire_status:08b}={self.fire_status}")

        self.get_logger().info(
            f"节点启动: publish_team={'是' if publish_team else '否'}, "
            f"队伍={'红方' if is_red else '蓝方'}, "
            f"publish_target={'是' if publish_target else '否'}, "
            f"target_priority={self.target_priority}, "
            f"火控={'启用' if enable_fire else '禁用'}, "
            f"自动开火={'是' if auto_fire else '否'}, "
            f"target_timeout={self.target_timeout_s:.1f}s, "
            f"diag_period={self.diag_period:.1f}s"
        )
        self.get_logger().info(
            f"控制话题: angles={self.control_angles_topic}, firecode={self.control_firecode_topic}"
        )

    @staticmethod
    def format_pub_nodes(pub_infos) -> List[str]:
        nodes: Set[str] = set()
        for info in pub_infos:
            namespace = info.node_namespace or "/"
            node_name = info.node_name or "unknown"
            full_name = f"{namespace.rstrip('/')}/{node_name}".replace("//", "/")
            nodes.add(full_name)
        return sorted(nodes)

    def publish_firecode(self, fire_status: int) -> None:
        fire_msg = UInt8()
        fire_msg.data = fire_status
        self.firecode_pub.publish(fire_msg)
        self.tx_fire_count += 1

    def publish_bt_target(self, target_id: int) -> None:
        if self.bt_target_pub is None:
            return
        target_msg = UInt8()
        target_msg.data = target_id
        self.bt_target_pub.publish(target_msg)
        self.tx_bt_target_count += 1

    def publish_team_color(self) -> None:
        if self.team_pub is None:
            return
        team_msg = Bool()
        team_msg.data = self.is_red
        self.team_pub.publish(team_msg)
        self.tx_team_count += 1

    def get_priority_rank(self, target_id: int) -> int:
        return self.priority_rank.get(target_id, len(self.target_priority))

    def pick_any_visible_target(self, msg: Armors) -> Optional[int]:
        best_target_id: Optional[int] = None
        best_key = None
        for armor in msg.armors:
            armor_type = int(armor.type)
            if not (TARGET_ID_MIN <= armor_type <= TARGET_ID_MAX):
                continue
            center_offset = (
                float(armor.distance_to_image_center)
                if float(armor.distance_to_image_center) > 0.0
                else math.inf
            )
            distance = float(armor.distance) if float(armor.distance) > 0.0 else math.inf
            candidate_key = (center_offset, distance, armor_type)
            if best_key is None or candidate_key < best_key:
                best_key = candidate_key
                best_target_id = armor_type
        return best_target_id

    def select_target_by_priority(self, msg: Armors, available_types: Set[int]) -> int:
        if not available_types:
            return self.selected_target_id

        current_target_id = self.selected_target_id
        preferred_target_id: Optional[int] = None
        for candidate in self.target_priority:
            if candidate in available_types:
                preferred_target_id = candidate
                break

        if current_target_id in available_types:
            if preferred_target_id is None:
                return current_target_id
            if self.get_priority_rank(current_target_id) <= self.get_priority_rank(preferred_target_id):
                return current_target_id

        if preferred_target_id is not None:
            return preferred_target_id

        any_visible_target_id = self.pick_any_visible_target(msg)
        if any_visible_target_id is not None:
            return any_visible_target_id

        return current_target_id

    def confirm_target_switch(self, desired_target_id: int, available_types: Set[int]) -> int:
        current_target_id = self.selected_target_id
        if desired_target_id == current_target_id:
            self.pending_target_id = None
            self.pending_target_frames = 0
            return current_target_id

        if current_target_id in available_types:
            if self.pending_target_id == desired_target_id:
                self.pending_target_frames += 1
            else:
                self.pending_target_id = desired_target_id
                self.pending_target_frames = 1

            if self.pending_target_frames < self.switch_confirm_frames:
                return current_target_id

        self.pending_target_id = None
        self.pending_target_frames = 0
        return desired_target_id

    def armors_callback(self, msg: Armors):
        self.rx_armors_count += 1
        available_types: Set[int] = set()
        for armor in msg.armors:
            armor_type = int(armor.type)
            if TARGET_ID_MIN <= armor_type <= TARGET_ID_MAX:
                available_types.add(armor_type)
        self.current_detected_types = available_types
        if available_types:
            self.last_armors_time = self.get_clock().now()

        desired_target_id = self.select_target_by_priority(msg, available_types)
        new_target_id = self.confirm_target_switch(desired_target_id, available_types)
        if new_target_id != self.selected_target_id:
            previous_target_id = self.selected_target_id
            self.selected_target_id = new_target_id
            self.get_logger().info(
                f"目标切换: {previous_target_id} -> {new_target_id}, "
                f"available={sorted(self.current_detected_types)}"
            )

    def target_callback(self, msg: Target):
        self.rx_target_count += 1
        self.last_yaw = msg.yaw
        self.last_pitch = msg.pitch
        self.last_status = bool(msg.status)

        gimbal_msg = GimbalAngles()
        gimbal_msg.header = msg.header
        gimbal_msg.yaw = msg.yaw
        gimbal_msg.pitch = msg.pitch
        self.gimbal_pub.publish(gimbal_msg)
        self.tx_angle_count += 1

        if self.enable_fire and self.auto_fire:
            self.last_fire_command = msg.status

    def emit_diag(self):
        angle_pub_infos = self.get_publishers_info_by_topic(self.control_angles_topic)
        fire_pub_infos = self.get_publishers_info_by_topic(self.control_firecode_topic)
        angle_pub_nodes = self.format_pub_nodes(angle_pub_infos)
        fire_pub_nodes = self.format_pub_nodes(fire_pub_infos)

        if len(angle_pub_nodes) > 1 and len(angle_pub_nodes) != self.last_warned_angle_pubs:
            self.get_logger().warn(
                f"{self.control_angles_topic} 存在多发布者({len(angle_pub_nodes)}): {', '.join(angle_pub_nodes)}"
            )
            self.last_warned_angle_pubs = len(angle_pub_nodes)
        elif len(angle_pub_nodes) <= 1:
            self.last_warned_angle_pubs = 0

        if len(fire_pub_nodes) > 1 and len(fire_pub_nodes) != self.last_warned_fire_pubs:
            self.get_logger().warn(
                f"{self.control_firecode_topic} 存在多发布者({len(fire_pub_nodes)}): {', '.join(fire_pub_nodes)}"
            )
            self.last_warned_fire_pubs = len(fire_pub_nodes)
        elif len(fire_pub_nodes) <= 1:
            self.last_warned_fire_pubs = 0

        if self.publish_target:
            bt_pub_infos = self.get_publishers_info_by_topic(self.bt_target_topic)
            bt_pub_nodes = self.format_pub_nodes(bt_pub_infos)
            if len(bt_pub_nodes) > 1 and len(bt_pub_nodes) != self.last_warned_bt_pubs:
                self.get_logger().warn(
                    f"{self.bt_target_topic} 存在多发布者({len(bt_pub_nodes)}): {', '.join(bt_pub_nodes)}"
                )
                self.last_warned_bt_pubs = len(bt_pub_nodes)
            elif len(bt_pub_nodes) <= 1:
                self.last_warned_bt_pubs = 0

        self.get_logger().info(
            f"diag: rx_target={self.rx_target_count} tx_angles={self.tx_angle_count} "
            f"rx_armors={self.rx_armors_count} tx_fire={self.tx_fire_count} tx_target={self.tx_bt_target_count} "
            f"target={self.selected_target_id} available={sorted(self.current_detected_types)} "
            f"last(yaw={self.last_yaw:.2f}, pitch={self.last_pitch:.2f}, status={int(self.last_status)})"
        )

    def timer_callback(self):
        self.publish_team_color()

        now = self.get_clock().now()
        stale_ns = (now - self.last_armors_time).nanoseconds
        if stale_ns > int(self.target_timeout_s * 1e9):
            self.current_detected_types = set()
        self.publish_bt_target(self.selected_target_id)

        if self.enable_fire and self.auto_fire:
            if self.last_fire_command:
                self.fire_status = 0b11 if self.fire_status == 0 else 0b00
            self.publish_firecode(self.fire_status)
        elif self.enable_fire:
            self.publish_firecode(self.fire_status)

        if (now - self.last_diag_time).nanoseconds >= int(self.diag_period * 1e9):
            self.last_diag_time = now
            self.emit_diag()


def main(args=None):
    parser = argparse.ArgumentParser(description="Target到云台角度映射节点")
    parser.add_argument(
        "--red",
        type=str2bool,
        default=True,
        help="是否为红方 (默认: true)",
    )
    parser.add_argument(
        "--target-id",
        type=parse_target_id,
        default=6,
        help="BT目标ID，作为初始目标和优先级列表为空时的默认目标 (默认: 6)",
    )
    parser.add_argument(
        "--target-priority",
        type=str,
        default="",
        help="目标优先级列表（逗号分隔），如: 6,3,4,5。优先打列表内当前可见目标；若都不可见，则改打其他可见目标。",
    )
    parser.add_argument(
        "--publish-team",
        type=str2bool,
        default=False,
        help="是否发布 /ly/me/is_team_red。默认 false（由 gimbal_driver/裁判系统提供）。",
    )
    parser.add_argument(
        "--publish-target",
        type=str2bool,
        default=True,
        help="是否发布 /ly/bt/target。默认 true。",
    )
    parser.add_argument(
        "--enable-fire",
        type=str2bool,
        default=True,
        help="是否启用火控功能 (默认: true)",
    )
    parser.add_argument(
        "--auto-fire",
        type=str2bool,
        default=True,
        help="是否自动开火 (默认: true)",
    )
    parser.add_argument(
        "--target-timeout",
        type=float,
        default=1.0,
        help="可见目标超时时间（秒）。超时后仅清空可见诊断，不强制回退固定目标（默认: 1.0）",
    )
    parser.add_argument(
        "--diag-period",
        type=float,
        default=1.0,
        help="诊断日志周期（秒，默认: 1.0）",
    )

    cli_args, ros_args = parser.parse_known_args(args=args)
    priority = parse_target_priority(cli_args.target_priority, cli_args.target_id)

    rclpy.init(args=ros_args)
    node = TargetToGimbalNode(
        is_red=cli_args.red,
        target_id=cli_args.target_id,
        target_priority=priority,
        publish_team=cli_args.publish_team,
        publish_target=cli_args.publish_target,
        enable_fire=cli_args.enable_fire,
        auto_fire=cli_args.auto_fire,
        target_timeout_s=cli_args.target_timeout,
        diag_period=cli_args.diag_period,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到Ctrl+C，节点退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
