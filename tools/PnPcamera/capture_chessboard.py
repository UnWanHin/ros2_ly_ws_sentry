#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SAVE_ROOT = SCRIPT_DIR / "save"
WINDOW_NAME = "PnP Chessboard Capture"


class ChessboardCaptureNode(Node):
    def __init__(self, topic: str, use_compressed: bool) -> None:
        super().__init__("pnp_chessboard_capture")
        self._use_compressed = use_compressed
        self._bridge = CvBridge() if (not use_compressed and CvBridge is not None) else None
        self._frame_lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None

        if use_compressed:
            self.create_subscription(CompressedImage, topic, self._compressed_callback, 10)
        else:
            if self._bridge is None:
                raise RuntimeError("cv_bridge is required for raw Image subscription.")
            self.create_subscription(Image, topic, self._raw_callback, 10)

        mode = "CompressedImage" if use_compressed else "Image"
        self.get_logger().info(f"Subscribed to {topic} ({mode})")

    def _compressed_callback(self, msg: CompressedImage) -> None:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._frame_lock:
            self._latest_frame = frame

    def _raw_callback(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self._frame_lock:
            self._latest_frame = frame

    def latest_frame(self) -> np.ndarray | None:
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture chessboard images from ROS2 topic into tools/PnPcamera/save."
    )
    parser.add_argument(
        "--topic",
        default="/ly/compressed/image",
        help="Input topic. Default: /ly/compressed/image",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Use sensor_msgs/Image subscription (default is CompressedImage).",
    )
    parser.add_argument(
        "--save-root",
        default=str(DEFAULT_SAVE_ROOT),
        help="Root save directory. Default: tools/PnPcamera/save",
    )
    parser.add_argument(
        "--session",
        default="",
        help="Session folder name under save-root. Default: auto timestamp.",
    )
    parser.add_argument(
        "--min-interval",
        type=float,
        default=0.2,
        help="Minimum save interval in seconds. Default: 0.2",
    )
    return parser.parse_args()


def create_session_dir(save_root: Path, session_name: str) -> Path:
    save_root.mkdir(parents=True, exist_ok=True)
    if not session_name:
        session_name = time.strftime("session_%Y%m%d_%H%M%S")
    session_dir = save_root / session_name
    session_dir.mkdir(parents=True, exist_ok=True)
    return session_dir


def draw_overlay(image: np.ndarray, save_dir: Path, saved_count: int) -> np.ndarray:
    canvas = image.copy()
    lines = [
        "Keys: [s]=save frame  [q/ESC]=quit",
        f"Saved: {saved_count}",
        f"Dir: {save_dir}",
    ]
    y = 30
    for line in lines:
        cv2.putText(canvas, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        y += 28
    return canvas


def blank_waiting_frame() -> np.ndarray:
    frame = np.zeros((480, 800, 3), dtype=np.uint8)
    cv2.putText(
        frame,
        "Waiting for topic frames...",
        (40, 240),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        "Press q to quit",
        (40, 280),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (180, 180, 180),
        2,
    )
    return frame


def main() -> int:
    args = parse_args()
    save_root = Path(args.save_root).expanduser().resolve()
    session_dir = create_session_dir(save_root, args.session)
    use_compressed = not args.raw

    print(f"[INFO] save root: {save_root}")
    print(f"[INFO] session dir: {session_dir}")
    print(f"[INFO] topic: {args.topic} ({'compressed' if use_compressed else 'raw'})")

    rclpy.init()
    try:
        node = ChessboardCaptureNode(args.topic, use_compressed)
    except Exception as exc:
        rclpy.shutdown()
        print(f"[ERROR] failed to initialize node: {exc}")
        return 1

    saved_count = 0
    last_save_ts = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            frame = node.latest_frame()
            if frame is None:
                display = blank_waiting_frame()
            else:
                display = draw_overlay(frame, session_dir, saved_count)

            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(10) & 0xFF

            if key in (27, ord("q")):
                break

            if key == ord("s") and frame is not None:
                now = time.time()
                if now - last_save_ts < args.min_interval:
                    continue
                filename = session_dir / f"img_{saved_count:04d}.png"
                ok = cv2.imwrite(str(filename), frame)
                if ok:
                    print(f"[SAVE] {filename}")
                    saved_count += 1
                    last_save_ts = now
                else:
                    print(f"[WARN] failed to save {filename}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    print(f"[INFO] done. total saved: {saved_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
