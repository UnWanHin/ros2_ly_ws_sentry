#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

from __future__ import annotations

import argparse
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="One-command offline decision start (fixed to regional profile)."
    )
    parser.add_argument(
        "--target",
        choices=("none", "predictor", "buff", "outpost"),
        default="predictor",
        help="Offline mock target source (default: predictor).",
    )
    parser.add_argument(
        "--trace",
        action="store_true",
        help="Enable trace output during offline decision run.",
    )
    parser.add_argument(
        "--no-view",
        action="store_true",
        help="Disable pygame live view (enabled by default).",
    )
    parser.add_argument(
        "--bt-config",
        default="regional_competition.json",
        help="BT config preset name/path under behavior_tree/Scripts/ConfigJson (default: regional_competition.json).",
    )
    parser.add_argument(
        "--web-port",
        type=int,
        default=0,
        help="Override decision_viz web stream port (default: YAML web_stream.port).",
    )
    parser.add_argument(
        "--match-duration-sec",
        type=int,
        default=420,
        help="Offline super confrontation match duration in seconds (default: 420).",
    )
    parser.add_argument(
        "--control-file",
        default="",
        help="Override viewer/mock control JSONL path (default: decision_viz.start default).",
    )
    parser.add_argument(
        "--bypass-is-start",
        action="store_true",
        help="Debug only: bypass /ly/game/is_start gate in offline mode.",
    )
    parser.add_argument(
        "--keep-tf-goal-bridge",
        action="store_true",
        help="Keep transformed goal-pos bridge in offline mode (default is official map coords only).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print command only.",
    )
    args = parser.parse_args(argv)
    if args.web_port < 0 or args.web_port > 65535:
        parser.error("--web-port must be in [0, 65535]")
    if args.match_duration_sec <= 0:
        parser.error("--match-duration-sec must be > 0")
    return args


def _collect_pids_by_pattern(pattern: str) -> list[int]:
    try:
        proc = subprocess.run(
            ["pgrep", "-f", pattern],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return []
    if proc.returncode not in (0, 1):
        return []
    pids: list[int] = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            pids.append(int(line))
        except ValueError:
            continue
    return pids


def _collect_pids_listening_on_port(port: int) -> list[int]:
    try:
        proc = subprocess.run(
            ["ss", "-ltnp"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return []
    if proc.returncode != 0:
        return []

    target = f":{port}"
    pids: set[int] = set()
    for line in proc.stdout.splitlines():
        if target not in line:
            continue
        marker = "pid="
        start = line.find(marker)
        while start != -1:
            start += len(marker)
            end = start
            while end < len(line) and line[end].isdigit():
                end += 1
            if end > start:
                try:
                    pids.add(int(line[start:end]))
                except ValueError:
                    pass
            start = line.find(marker, end)
    return sorted(pids)


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _terminate_pids(pids: list[int], timeout_sec: float = 1.2) -> list[int]:
    if not pids:
        return []
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.monotonic() + max(0.1, timeout_sec)
    while time.monotonic() < deadline:
        alive = [pid for pid in pids if _alive(pid)]
        if not alive:
            return []
        time.sleep(0.05)
    alive = [pid for pid in pids if _alive(pid)]
    for pid in alive:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(0.05)
    return [pid for pid in alive if _alive(pid)]


def cleanup_stale_viewers(web_port: int = 9000) -> None:
    this_pid = os.getpid()
    candidates: set[int] = set()
    for pattern in ("decision_viz.main", "decision-viz "):
        for pid in _collect_pids_by_pattern(pattern):
            if pid != this_pid:
                candidates.add(pid)
    for pid in _collect_pids_listening_on_port(web_port):
        if pid != this_pid:
            candidates.add(pid)

    if not candidates:
        return
    killed = sorted(candidates)
    still_alive = _terminate_pids(killed)
    if still_alive:
        print(f"warning: stale viewer/port processes still alive: {still_alive}", file=sys.stderr)
    else:
        print(f"cleaned stale viewer processes: {killed}")


def load_default_web_port(root: Path) -> int:
    config_path = root / "src" / "decision_viz" / "config" / "default.yaml"
    if not config_path.exists():
        return 9000
    try:
        import yaml  # type: ignore
    except ImportError:
        return 9000
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except Exception:
        return 9000
    if not isinstance(data, dict):
        return 9000
    web_cfg = data.get("web_stream", {})
    if not isinstance(web_cfg, dict):
        return 9000
    try:
        port = int(web_cfg.get("port", 9000))
    except (TypeError, ValueError):
        return 9000
    if port <= 0 or port > 65535:
        return 9000
    return port


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    root = Path(__file__).resolve().parents[2]
    install_setup = root / "install" / "setup.bash"
    ros_setup = Path("/opt/ros/humble/setup.bash")

    if not ros_setup.exists():
        print(f"missing ROS setup: {ros_setup}", file=sys.stderr)
        return 2
    if not install_setup.exists():
        print(f"missing workspace setup: {install_setup}", file=sys.stderr)
        print(
            "build first: colcon build --packages-select behavior_tree auto_aim_common gimbal_driver decision_viz",
            file=sys.stderr,
        )
        return 2

    decision_cmd = [
        "python3",
        "-m",
        "decision_viz.start",
        "--offline-decision",
        "--mode",
        "regional",
        "--entry",
        "nogate",
        "--bt-config",
        args.bt_config,
        "--mock-target",
        args.target,
    ]
    if not args.no_view:
        decision_cmd.append("--live-view")
    if args.web_port > 0:
        decision_cmd.extend(["--live-web-port", str(args.web_port)])
    if args.match_duration_sec > 0:
        decision_cmd.extend(["--match-duration-sec", str(args.match_duration_sec)])
    if args.control_file.strip():
        decision_cmd.extend(["--control-file", str(Path(args.control_file).expanduser().resolve())])
    if args.bypass_is_start:
        decision_cmd.append("--bypass-is-start")
    if args.keep_tf_goal_bridge:
        decision_cmd.append("--keep-tf-goal-bridge")
    if args.trace:
        decision_cmd.append("--trace-on")
    if args.dry_run:
        decision_cmd.append("--dry-run")

    env_prefix = [
        f"source {shlex.quote(str(ros_setup))}",
        f"source {shlex.quote(str(install_setup))}",
    ]
    run_cmd = " ".join(shlex.quote(item) for item in decision_cmd)
    shell_cmd = " && ".join(env_prefix + [f"PYTHONPATH={shlex.quote(str(root / 'src' / 'decision_viz'))} {run_cmd}"])

    print("offline decision starter")
    print(f"workspace: {root}")
    print(f"command: {shell_cmd}")

    if args.dry_run:
        return 0

    if not args.no_view:
        web_port = args.web_port if args.web_port > 0 else load_default_web_port(root)
        cleanup_stale_viewers(web_port=web_port)

    return subprocess.run(["bash", "-lc", shell_cmd], check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
