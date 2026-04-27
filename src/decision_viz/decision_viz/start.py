from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from .config import repo_root


DEFAULT_BT_CONFIG_BY_MODE = {
    "regional": "regional_competition.json",
    "league": "league_competition.json",
    "showcase": "showcase_competition.json",
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record behavior_tree decision traces with start.sh and optional config indexing "
            "from src/behavior_tree/Scripts/ConfigJson."
        )
    )
    parser.add_argument(
        "--entry",
        choices=("nogate", "gated"),
        default="nogate",
        help="Entry forwarded to scripts/start.sh (default: nogate).",
    )
    parser.add_argument(
        "--mode",
        choices=("regional", "league", "showcase"),
        default="league",
        help="Competition mode forwarded to scripts/start.sh (default: league).",
    )
    parser.add_argument(
        "--bt-config",
        default="",
        help=(
            "Behavior-tree config preset name or path. Examples: league_competition.json, "
            "Scripts/ConfigJson/chase_only_competition.json, /abs/path/custom.json"
        ),
    )
    parser.add_argument(
        "--trace",
        default="",
        help="Trace output JSONL path. Default: log/decision_trace_<timestamp>.jsonl",
    )
    parser.add_argument(
        "--every",
        type=int,
        default=5,
        help="decision_trace_every_n_ticks value (default: 5).",
    )
    parser.add_argument(
        "--offline-decision",
        action="store_true",
        help="Run decision offline: behavior_tree only, with built-in mock input topics.",
    )
    parser.add_argument(
        "--trace-on",
        action="store_true",
        help="Force trace recording in --offline-decision mode.",
    )
    parser.add_argument(
        "--mock-team",
        choices=("red", "blue"),
        default="red",
        help="Mock team color for --offline-decision (default: red).",
    )
    parser.add_argument(
        "--mock-target",
        choices=("none", "predictor", "buff", "outpost"),
        default="none",
        help="Mock target source for --offline-decision (default: none).",
    )
    parser.add_argument(
        "--mock-hz",
        type=float,
        default=20.0,
        help="Mock input publish rate for --offline-decision (default: 20).",
    )
    parser.add_argument(
        "--mock-time-left",
        type=int,
        default=420,
        help="Mock /ly/game/time_left value for --offline-decision (default: 420).",
    )
    parser.add_argument(
        "--mock-ammo",
        type=int,
        default=200,
        help="Mock /ly/me/ammo_left value for --offline-decision (default: 200).",
    )
    parser.add_argument(
        "--mock-posture",
        type=int,
        default=1,
        help="Mock /ly/gimbal/posture value for --offline-decision (default: 1).",
    )
    parser.add_argument(
        "--mock-yaw",
        type=float,
        default=0.0,
        help="Mock /ly/gimbal/angles yaw (default: 0).",
    )
    parser.add_argument(
        "--mock-pitch",
        type=float,
        default=0.0,
        help="Mock /ly/gimbal/angles pitch (default: 0).",
    )
    parser.add_argument(
        "--bypass-is-start",
        action="store_true",
        help="Debug only: bypass /ly/game/is_start gate in offline mode.",
    )
    parser.add_argument(
        "--keep-tf-goal-bridge",
        action="store_true",
        help=(
            "Offline mode only: keep NaviSetting.UseTfGoalBridge from source config. "
            "Default is to force official map coordinates (UseTfGoalBridge=false)."
        ),
    )
    parser.add_argument(
        "--match-duration-sec",
        type=int,
        default=420,
        help="Offline super confrontation match duration in seconds (default: 420).",
    )
    parser.add_argument(
        "--control-file",
        default="/tmp/decision_viz_match_control.jsonl",
        help="JSONL command channel between live viewer and mock inputs (default: /tmp/decision_viz_match_control.jsonl).",
    )
    parser.add_argument(
        "--list-configs",
        action="store_true",
        help="List available presets under src/behavior_tree/Scripts/ConfigJson and exit.",
    )
    parser.add_argument(
        "--include-legacy",
        action="store_true",
        help="Include src/behavior_tree/Scripts/ConfigJson/legacy presets in --list-configs.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the final start.sh command without executing.",
    )
    parser.add_argument(
        "--play",
        action="store_true",
        help="Open pygame viewer on the trace file after recording exits.",
    )
    parser.add_argument(
        "--live-view",
        action="store_true",
        help="Open pygame viewer while decision is running (follow growing trace file).",
    )
    parser.add_argument(
        "--live-follow-poll",
        type=float,
        default=0.25,
        help="Follow poll interval for --live-view (default: 0.25s).",
    )
    parser.add_argument(
        "--live-web-host",
        default="",
        help="Override viewer web host in live view (default: YAML web_stream.host).",
    )
    parser.add_argument(
        "--live-web-port",
        type=int,
        default=0,
        help="Override viewer web port in live view (default: YAML web_stream.port).",
    )
    parser.add_argument(
        "--live-web-fps",
        type=float,
        default=0.0,
        help="Override viewer web stream fps in live view (default: YAML web_stream.fps).",
    )
    parser.add_argument(
        "--live-web-jpeg-quality",
        type=int,
        default=0,
        help="Override viewer web JPEG quality in live view (default: YAML web_stream.jpeg_quality).",
    )
    parser.add_argument(
        "extra_launch_args",
        nargs=argparse.REMAINDER,
        help="Extra launch args passed through to scripts/start.sh. Put them after '--'.",
    )
    args = parser.parse_args(argv)
    if args.every < 1:
        parser.error("--every must be >= 1")
    if args.mock_hz <= 0:
        parser.error("--mock-hz must be > 0")
    if args.live_follow_poll <= 0:
        parser.error("--live-follow-poll must be > 0")
    if args.live_web_port < 0 or args.live_web_port > 65535:
        parser.error("--live-web-port must be in [0, 65535]")
    if args.live_web_fps < 0:
        parser.error("--live-web-fps must be >= 0")
    if args.live_web_jpeg_quality < 0 or args.live_web_jpeg_quality > 100:
        parser.error("--live-web-jpeg-quality must be in [0, 100]")
    if args.match_duration_sec <= 0:
        parser.error("--match-duration-sec must be > 0")
    if args.play and args.live_view:
        parser.error("--play and --live-view are mutually exclusive")
    return args


def behavior_tree_root(root: Path) -> Path:
    return (root / "src" / "behavior_tree").resolve()


def config_dir(root: Path) -> Path:
    return (behavior_tree_root(root) / "Scripts" / "ConfigJson").resolve()


def normalize_extra_launch_args(raw: list[str]) -> list[str]:
    if raw and raw[0] == "--":
        return raw[1:]
    return raw


def iter_config_paths(config_root: Path, include_legacy: bool) -> list[Path]:
    paths = sorted(config_root.glob("*.json"))
    if include_legacy:
        paths.extend(sorted((config_root / "legacy").glob("*.json")))
    return paths


def choose_trace_path(root: Path, configured: str) -> Path:
    if configured.strip():
        raw = Path(configured).expanduser()
        if raw.is_absolute():
            return raw.resolve()
        return (root / raw).resolve()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (root / "log" / f"decision_trace_{stamp}.jsonl").resolve()


def resolve_bt_config_path(root: Path, mode: str, configured: str) -> tuple[Path, str]:
    bt_root = behavior_tree_root(root)
    cfg_root = config_dir(root)

    candidates: list[Path] = []
    if not configured.strip():
        default_name = DEFAULT_BT_CONFIG_BY_MODE[mode]
        candidates.append(cfg_root / default_name)
    else:
        raw = Path(configured).expanduser()
        if raw.is_absolute():
            candidates.append(raw)
        else:
            candidates.append((root / raw).resolve())
            candidates.append((bt_root / raw).resolve())
            candidates.append((cfg_root / raw).resolve())
            candidates.append((cfg_root / raw.name).resolve())
            if raw.suffix != ".json":
                candidates.append((cfg_root / f"{raw.name}.json").resolve())
                candidates.append((cfg_root / "legacy" / f"{raw.name}.json").resolve())
            candidates.append((cfg_root / "legacy" / raw.name).resolve())

    seen: set[Path] = set()
    existing: Path | None = None
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.exists() and resolved.is_file():
            existing = resolved
            break

    if existing is None:
        attempted = "\n".join(str(c) for c in seen)
        raise FileNotFoundError(f"Cannot find bt config. Tried:\n{attempted}")

    try:
        launch_rel = existing.relative_to(bt_root).as_posix()
        launch_value = launch_rel
    except ValueError:
        launch_value = existing.as_posix()

    return existing, launch_value


def print_configs(root: Path, include_legacy: bool) -> int:
    cfg_root = config_dir(root)
    if not cfg_root.exists():
        print(f"config dir not found: {cfg_root}", file=sys.stderr)
        return 2
    print(f"Config root: {cfg_root}")
    for item in iter_config_paths(cfg_root, include_legacy):
        rel = item.relative_to(cfg_root).as_posix()
        print(rel)
    return 0


def has_launch_arg(args: list[str], key: str) -> bool:
    prefix = f"{key}:="
    return any(item.startswith(prefix) for item in args)


def should_enable_trace(args: argparse.Namespace) -> bool:
    if args.offline_decision:
        return args.trace_on or args.play or args.live_view or bool(args.trace.strip())
    return True


def build_start_command(
    root: Path,
    entry: str,
    mode: str,
    bt_config_launch: str,
    debug_bypass_is_start: bool,
    trace_enabled: bool,
    trace_path: Path | None,
    every: int,
    offline_decision: bool,
    extra_launch_args: list[str],
) -> list[str]:
    if offline_decision:
        command: list[str] = [
            str((root / "scripts" / "launch" / "start_sentry_all.sh").resolve()),
            "--mode",
            mode,
            f"bt_config_file:={bt_config_launch}",
        ]
    else:
        command = [
            str((root / "scripts" / "start.sh").resolve()),
            entry,
            "--mode",
            mode,
            f"bt_config_file:={bt_config_launch}",
        ]

    if offline_decision:
        offline_defaults = (
            ("offline", "true"),
            ("use_gimbal", "false"),
            ("use_detector", "false"),
            ("use_tracker", "false"),
            ("use_predictor", "false"),
            ("use_outpost", "false"),
            ("use_buff", "false"),
            ("use_behavior_tree", "true"),
            ("debug_bypass_is_start", "true" if debug_bypass_is_start else "false"),
        )
        for key, value in offline_defaults:
            if not has_launch_arg(extra_launch_args, key):
                command.append(f"{key}:={value}")

    if trace_enabled and trace_path is not None:
        if not has_launch_arg(extra_launch_args, "decision_trace_enabled"):
            command.append("decision_trace_enabled:=true")
        if not has_launch_arg(extra_launch_args, "decision_trace_file"):
            command.append(f"decision_trace_file:={trace_path.as_posix()}")
        if not has_launch_arg(extra_launch_args, "decision_trace_every_n_ticks"):
            command.append(f"decision_trace_every_n_ticks:={every}")

    command.extend(extra_launch_args)
    return command


def build_mock_command(root: Path, args: argparse.Namespace) -> tuple[list[str], str]:
    system_python = Path("/usr/bin/python3")
    python_exec = str(system_python if system_python.exists() else Path(sys.executable))
    python_args = [
        python_exec,
        "-m",
        "decision_viz.mock_inputs",
        "--team",
        args.mock_team,
        "--target-source",
        args.mock_target,
        "--hz",
        str(args.mock_hz),
        "--time-left",
        str(args.mock_time_left),
        "--ammo-left",
        str(args.mock_ammo),
        "--posture",
        str(args.mock_posture),
        "--yaw",
        str(args.mock_yaw),
        "--pitch",
        str(args.mock_pitch),
        "--simulate-match",
        "true",
        "--start-running",
        "false",
        "--match-duration-sec",
        str(args.match_duration_sec),
    ]
    if str(args.control_file).strip():
        python_args.extend(["--control-file", str(Path(args.control_file).expanduser().resolve())])

    setup_cmds: list[str] = []
    ros_setup = Path("/opt/ros/humble/setup.bash")
    ws_setup = root / "install" / "setup.bash"
    if ros_setup.exists():
        setup_cmds.append(f"source {shlex.quote(str(ros_setup))}")
    if ws_setup.exists():
        setup_cmds.append(f"source {shlex.quote(str(ws_setup))}")
    setup_cmds.append("mkdir -p /tmp/ros2_logs")
    setup_cmds.append("export ROS_LOG_DIR=/tmp/ros2_logs")
    setup_cmds.append(
        f"export PYTHONPATH={shlex.quote(str((root / 'src' / 'decision_viz').resolve()))}:$PYTHONPATH"
    )

    quoted_python = " ".join(shlex.quote(item) for item in python_args)
    setup_cmds.append(quoted_python)
    shell_cmd = " && ".join(setup_cmds)
    return (["bash", "-lc", shell_cmd], shell_cmd)


def build_offline_bt_config(root: Path, source_config: Path) -> Path:
    with source_config.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"invalid config root (expect object): {source_config}")

    navi = data.get("NaviSetting")
    if not isinstance(navi, dict):
        navi = {}
        data["NaviSetting"] = navi
    navi["UseTfGoalBridge"] = False

    out_dir = (root / "log" / "decision_viz").resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{source_config.stem}.offline_official_goal_pos.json"
    with out_path.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return out_path


def run_viewer(trace_path: Path) -> int:
    cmd = [sys.executable, "-m", "decision_viz.main", trace_path.as_posix()]
    return subprocess.run(cmd, check=False).returncode


def start_live_viewer(
    trace_path: Path,
    follow_poll: float,
    web_host: str,
    web_port: int,
    web_fps: float,
    web_jpeg_quality: int,
    control_file: str,
    match_duration_sec: int,
) -> subprocess.Popen[bytes]:
    cmd = [
        sys.executable,
        "-m",
        "decision_viz.main",
        trace_path.as_posix(),
        "--follow",
        "--follow-poll",
        f"{follow_poll}",
        "--follow-wait",
        "600",
    ]
    if web_host.strip():
        cmd.extend(["--web-host", web_host.strip()])
    if web_port > 0:
        cmd.extend(["--web-port", str(web_port)])
    if web_fps > 0:
        cmd.extend(["--web-fps", f"{web_fps}"])
    if web_jpeg_quality > 0:
        cmd.extend(["--web-jpeg-quality", str(web_jpeg_quality)])
    if str(control_file).strip():
        cmd.extend(["--control-file", str(Path(control_file).expanduser().resolve())])
    if match_duration_sec > 0:
        cmd.extend(["--match-duration-sec", str(match_duration_sec)])
    return subprocess.Popen(cmd)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    root = repo_root().resolve()

    if args.list_configs:
        return print_configs(root, args.include_legacy)

    extra_launch_args = normalize_extra_launch_args(args.extra_launch_args)
    try:
        config_file, bt_config_launch = resolve_bt_config_path(root, args.mode, args.bt_config)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if args.offline_decision and not args.keep_tf_goal_bridge:
        try:
            offline_config = build_offline_bt_config(root, config_file)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            print(f"failed to prepare offline BT config: {exc}", file=sys.stderr)
            return 2
        config_file = offline_config
        bt_config_launch = offline_config.as_posix()
        print(f"offline BT config (official map goal_pos): {offline_config}")

    trace_enabled = should_enable_trace(args)
    trace_path: Path | None = None
    if trace_enabled:
        trace_path = choose_trace_path(root, args.trace)
        trace_path.parent.mkdir(parents=True, exist_ok=True)

    start_cmd = build_start_command(
        root=root,
        entry=args.entry,
        mode=args.mode,
        bt_config_launch=bt_config_launch,
        debug_bypass_is_start=args.bypass_is_start,
        trace_enabled=trace_enabled,
        trace_path=trace_path,
        every=args.every,
        offline_decision=args.offline_decision,
        extra_launch_args=extra_launch_args,
    )

    print(f"decision mode: {args.mode}")
    print(f"bt config: {config_file}")
    if args.offline_decision:
        print("run profile: offline decision test (behavior_tree + mock inputs)")
    else:
        print("run profile: stack run")
    print(f"trace enabled: {str(trace_enabled).lower()}")
    if trace_path is not None:
        print(f"trace output: {trace_path}")
    print(f"start command: {' '.join(start_cmd)}")
    mock_cmd: list[str] | None = None
    mock_cmd_desc = ""
    control_path: Path | None = None
    if args.offline_decision:
        if str(args.control_file).strip():
            control_path = Path(args.control_file).expanduser().resolve()
            print(f"control file: {control_path}")
        mock_cmd, mock_cmd_desc = build_mock_command(root, args)
        print(f"mock command: {mock_cmd_desc}")

    if args.dry_run:
        return 0

    mock_proc: subprocess.Popen[bytes] | None = None
    viewer_proc: subprocess.Popen[bytes] | None = None
    try:
        if control_path is not None:
            control_path.parent.mkdir(parents=True, exist_ok=True)
            if control_path.exists():
                try:
                    control_path.unlink()
                except OSError:
                    pass
        if mock_cmd is not None:
            mock_proc = subprocess.Popen(mock_cmd)
            # If mock exits immediately, offline inputs are not being published.
            time.sleep(0.5)
            mock_rc = mock_proc.poll()
            if mock_rc is not None:
                print(
                    (
                        "mock inputs exited early "
                        f"(rc={mock_rc}); offline /ly/gimbal/angles may be missing. "
                        "Likely Python/ROS env mismatch (e.g. conda)."
                    ),
                    file=sys.stderr,
                )
                return 2
        if args.live_view:
            if trace_path is None:
                print("trace is disabled but --live-view requested", file=sys.stderr)
                return 2
            if args.live_web_port > 0:
                print(
                    "live web stream: "
                    f"http://127.0.0.1:{args.live_web_port}/ "
                    f"(LAN: http://<your-ip>:{args.live_web_port}/)"
                )
            else:
                print("live web stream: use YAML web_stream.port (default 9000)")
            viewer_proc = start_live_viewer(
                trace_path,
                args.live_follow_poll,
                args.live_web_host,
                args.live_web_port,
                args.live_web_fps,
                args.live_web_jpeg_quality,
                args.control_file,
                args.match_duration_sec,
            )
            # Give viewer a moment to start and enter follow wait state.
            time.sleep(0.5)
            viewer_rc = viewer_proc.poll()
            if viewer_rc is not None:
                print(
                    f"live viewer exited early (rc={viewer_rc}); check pygame/display environment.",
                    file=sys.stderr,
                )
                return 2
        rc = subprocess.run(start_cmd, check=False).returncode
        if rc != 0:
            return rc

        if args.play:
            if trace_path is None:
                print("trace is disabled but --play requested", file=sys.stderr)
                return 2
            if not trace_path.exists():
                print(f"trace file not found after run: {trace_path}", file=sys.stderr)
                return 2
            return run_viewer(trace_path)
        return 0
    finally:
        if viewer_proc is not None and viewer_proc.poll() is None:
            viewer_proc.terminate()
            try:
                viewer_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                viewer_proc.kill()
                viewer_proc.wait(timeout=3)
        if mock_proc is not None and mock_proc.poll() is None:
            mock_proc.terminate()
            try:
                mock_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                mock_proc.kill()
                mock_proc.wait(timeout=3)


if __name__ == "__main__":
    raise SystemExit(main())
