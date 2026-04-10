#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import csv
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from auto_aim_common.msg import BuffDebug
from gimbal_driver.msg import GimbalAngles
from std_msgs.msg import UInt8


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    text = str(value).strip().lower()
    return text in {"1", "true", "yes", "on", "y"}


class BuffShootingTableCalibNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "buff_shooting_table_calib",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.calib_mode = str(self.get_param_compat("calib_mode", "static")).strip().lower()
        if self.calib_mode not in {"static", "periodic", "all"}:
            self.get_logger().warn(
                f"Invalid calib_mode={self.calib_mode}, fallback to static (supported: static/periodic/all)"
            )
            self.calib_mode = "static"

        default_record_dir = str((Path.home() / "workspace" / "record").resolve())
        self.record_dir = str(self.get_param_compat("record_dir", default_record_dir))
        self.csv_strategy = str(self.get_param_compat("csv_strategy", "new")).strip().lower()
        self.csv_path_param = str(self.get_param_compat("csv_path", "")).strip()

        self.debug_topic = str(self.get_param_compat("buff_debug_topic", "/ly/buff/debug"))
        self.gimbal_topic = str(self.get_param_compat("gimbal_topic", "/ly/gimbal/angles"))
        self.firecode_topic = str(self.get_param_compat("firecode_topic", "/ly/gimbal/firecode"))

        self.require_valid_debug = parse_bool(self.get_param_compat("require_valid_debug", True))
        self.sample_on_rising_edge = parse_bool(self.get_param_compat("sample_on_rising_edge", True))
        self.min_sample_interval_sec = float(self.get_param_compat("min_sample_interval_sec", 0.08))
        self.min_sample_interval_ns = int(max(0.0, self.min_sample_interval_sec) * 1e9)

        self.latest_debug: Optional[BuffDebug] = None
        self.latest_gimbal: Optional[GimbalAngles] = None
        self.last_fire_active = False
        self.last_sample_ns = 0

        self.sample_count = 0
        self.drop_no_debug = 0
        self.drop_no_gimbal = 0
        self.drop_invalid_debug = 0
        self.drop_mode_filter = 0
        self.drop_interval = 0

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = self.prepare_csv_file(self.record_dir, self.csv_strategy, self.csv_path_param)

        self.debug_sub = self.create_subscription(BuffDebug, self.debug_topic, self.on_debug, 30)
        self.gimbal_sub = self.create_subscription(GimbalAngles, self.gimbal_topic, self.on_gimbal, 30)
        self.firecode_sub = self.create_subscription(UInt8, self.firecode_topic, self.on_firecode, 30)

        self.info_timer = self.create_timer(5.0, self.print_stats)

        self.get_logger().info(f"buff calib mode: {self.calib_mode}")
        self.get_logger().info(f"buff debug topic: {self.debug_topic}")
        self.get_logger().info(f"gimbal topic: {self.gimbal_topic}")
        self.get_logger().info(f"firecode topic: {self.firecode_topic}")
        self.get_logger().info(f"csv output: {self.csv_path}")

    def get_param_compat(self, key: str, default):
        # Prefer prefixed key for consistency with launch-time override style.
        bare_value = self.declare_and_get(key, default)
        prefixed_key = f"buff_shooting_table_calib.{key}"
        prefixed_value = self.declare_and_get(prefixed_key, bare_value)
        return prefixed_value

    def declare_and_get(self, key: str, default):
        if not self.has_parameter(key):
            self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def prepare_csv_file(self, record_dir: str, csv_strategy: str, csv_path_param: str) -> Path:
        record_path = Path(record_dir).expanduser().resolve()
        record_path.mkdir(parents=True, exist_ok=True)

        selected: Optional[Path] = None
        if csv_path_param:
            selected = Path(csv_path_param).expanduser()
            if not selected.is_absolute():
                selected = record_path / selected
            selected = selected.resolve()
        elif csv_strategy == "latest":
            latest_candidates = sorted(record_path.glob("buff_shooting_table_*.csv"), key=lambda p: p.stat().st_mtime)
            if latest_candidates:
                selected = latest_candidates[-1]

        if selected is None:
            selected = record_path / f"buff_shooting_table_{int(time.time())}.csv"

        should_write_header = True
        if selected.exists() and selected.stat().st_size > 0:
            should_write_header = False

        self.csv_file = selected.open("a", encoding="utf-8", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        if should_write_header:
            self.csv_writer.writerow(
                [
                    "timestamp",
                    "mode",
                    "status",
                    "firecode",
                    "target_yaw",
                    "target_pitch",
                    "gimbal_yaw",
                    "gimbal_pitch",
                    "relative_yaw",
                    "relative_pitch",
                    "rotation_angle",
                    "distance_m",
                    "height_m",
                    "target_x_m",
                    "target_y_m",
                    "target_z_m",
                ]
            )
            self.csv_file.flush()

        return selected

    def on_debug(self, msg: BuffDebug) -> None:
        self.latest_debug = msg

    def on_gimbal(self, msg: GimbalAngles) -> None:
        self.latest_gimbal = msg

    def mode_allowed(self, mode: int) -> bool:
        if self.calib_mode == "all":
            return True
        if self.calib_mode == "periodic":
            return mode == 2
        # static mode: keep all valid modes by default; offline fitter can filter mode later.
        return True

    def on_firecode(self, msg: UInt8) -> None:
        fire_active = (int(msg.data) & 0b11) != 0
        should_sample = fire_active
        if self.sample_on_rising_edge:
            should_sample = fire_active and not self.last_fire_active
        self.last_fire_active = fire_active

        if not should_sample:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.min_sample_interval_ns > 0 and (now_ns - self.last_sample_ns) < self.min_sample_interval_ns:
            self.drop_interval += 1
            return

        if self.latest_debug is None:
            self.drop_no_debug += 1
            return
        if self.latest_gimbal is None:
            self.drop_no_gimbal += 1
            return
        if self.require_valid_debug and (not self.latest_debug.status):
            self.drop_invalid_debug += 1
            return
        if not self.mode_allowed(int(self.latest_debug.mode)):
            self.drop_mode_filter += 1
            return

        debug = self.latest_debug
        gimbal = self.latest_gimbal

        relative_yaw = float(gimbal.yaw - debug.target_yaw)
        relative_pitch = float(gimbal.pitch - debug.target_pitch)

        if debug.header.stamp.sec == 0 and debug.header.stamp.nanosec == 0:
            timestamp = now_ns * 1e-9
        else:
            timestamp = float(debug.header.stamp.sec) + float(debug.header.stamp.nanosec) * 1e-9

        self.csv_writer.writerow(
            [
                f"{timestamp:.6f}",
                int(debug.mode),
                bool(debug.status),
                int(msg.data),
                float(debug.target_yaw),
                float(debug.target_pitch),
                float(gimbal.yaw),
                float(gimbal.pitch),
                relative_yaw,
                relative_pitch,
                float(debug.rotation_angle),
                float(debug.distance_m),
                float(debug.height_m),
                float(debug.target_x_m),
                float(debug.target_y_m),
                float(debug.target_z_m),
            ]
        )
        self.csv_file.flush()

        self.last_sample_ns = now_ns
        self.sample_count += 1

    def print_stats(self) -> None:
        self.get_logger().info(
            "samples=%d drop(no_debug=%d no_gimbal=%d invalid=%d mode=%d interval=%d)",
            self.sample_count,
            self.drop_no_debug,
            self.drop_no_gimbal,
            self.drop_invalid_debug,
            self.drop_mode_filter,
            self.drop_interval,
        )

    def destroy_node(self) -> bool:
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BuffShootingTableCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
