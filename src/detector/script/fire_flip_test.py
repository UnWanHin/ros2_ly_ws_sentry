#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
import os
import signal
import subprocess
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


def str2bool(value: str) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in ('1', 'true', 'yes', 'y', 'on'):
        return True
    if lowered in ('0', 'false', 'no', 'n', 'off'):
        return False
    raise argparse.ArgumentTypeError(f'invalid bool value: {value}')


class FireFlipTestNode(Node):
    def __init__(self, fire_hz: float, diag_period: float = 1.0):
        super().__init__('fire_flip_test')
        self.fire_pub = self.create_publisher(UInt8, '/ly/control/firecode', 10)
        self.fire_status = 0
        self.tx_count = 0
        self.diag_period = max(diag_period, 0.2)
        self.last_diag_ts = self.get_clock().now()

        self.get_logger().warn(
            f'Fire flip test started: /ly/control/firecode will toggle at {fire_hz:.2f} Hz')

        # 啟動即翻轉一次，方便硬件測試不等待第一個 timer 週期
        self._on_timer()

        period = 1.0 / max(fire_hz, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

    @staticmethod
    def _format_pub_nodes(pub_infos) -> List[str]:
        nodes = set()
        for info in pub_infos:
            namespace = info.node_namespace or '/'
            node_name = info.node_name or 'unknown'
            full_name = f"{namespace.rstrip('/')}/{node_name}".replace('//', '/')
            nodes.add(full_name)
        return sorted(nodes)

    def _publish_fire(self, value: int):
        msg = UInt8()
        msg.data = value
        self.fire_pub.publish(msg)
        self.tx_count += 1

    def _on_timer(self):
        self.fire_status = 0b11 if self.fire_status == 0 else 0b00
        self._publish_fire(self.fire_status)
        now = self.get_clock().now()
        if (now - self.last_diag_ts).nanoseconds >= int(self.diag_period * 1e9):
            self.last_diag_ts = now
            pub_infos = self.get_publishers_info_by_topic('/ly/control/firecode')
            pub_nodes = self._format_pub_nodes(pub_infos)
            if len(pub_nodes) > 1:
                self.get_logger().warn(
                    f'/ly/control/firecode 多发布者({len(pub_nodes)}): {", ".join(pub_nodes)}')
            self.get_logger().info(f'diag: tx_fire={self.tx_count}, last_fire_status={self.fire_status}')

    def stop_safely(self):
        # 退出前清零，避免殘留開火狀態
        try:
            if rclpy.ok():
                self._publish_fire(0)
                self.get_logger().info('Fire flip test stopping, firecode reset to 0')
        except Exception as exc:
            self.get_logger().warn(f'Fire flip stop_safely skipped: {exc}')


def _start_detector_process(params_file: str | None, warmup_sec: float) -> subprocess.Popen:
    cmd = ['ros2', 'run', 'detector', 'detector_node']
    if params_file:
        cmd += ['--ros-args', '--params-file', params_file]

    print('[fire_flip_test] starting detector:', ' '.join(cmd), flush=True)
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    time.sleep(max(warmup_sec, 0.0))
    return proc


def _stop_process_tree(proc: subprocess.Popen):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=3.0)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def main(args=None):
    parser = argparse.ArgumentParser(
        description='實機火控翻轉測試：可選擇拉起 detector，並持續翻轉 firecode')
    parser.add_argument('--fire-hz', type=float, default=8.0,
                        help='火控翻轉頻率 (Hz), 預設 8.0')
    parser.add_argument('--start-detector',
                        type=str2bool,
                        default=False,
                        help='是否同時拉起 detector_node (預設: false)')
    parser.add_argument('--params-file', type=str, default='',
                        help='可選: detector params yaml 路徑')
    parser.add_argument('--warmup-sec', type=float, default=1.5,
                        help='拉起 detector 後等待秒數 (預設: 1.5)')
    parser.add_argument('--diag-period', type=float, default=1.0,
                        help='诊断日志周期（秒，默认 1.0）')
    cli, ros_args = parser.parse_known_args(args=args)

    detector_proc = None
    if cli.start_detector:
        detector_proc = _start_detector_process(
            cli.params_file if cli.params_file else None,
            cli.warmup_sec,
        )

    rclpy.init(args=ros_args)
    node = FireFlipTestNode(cli.fire_hz, cli.diag_period)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.stop_safely()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _stop_process_tree(detector_proc)


if __name__ == '__main__':
    main()
