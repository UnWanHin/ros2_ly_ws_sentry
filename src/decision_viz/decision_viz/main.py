from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from .config import goals_by_id, load_config, load_plugin_points, resolve_path
from .control_bus import append_command
from .trace import build_changes, load_trace, load_trace_incremental
from .validation import format_validation, validate_records
from .viewer import Viewer


def import_pygame():
    try:
        import pygame  # type: ignore
    except ImportError:
        print(
            "pygame is required. Install it with: python3 -m pip install -r src/decision_viz/requirements.txt",
            file=sys.stderr,
        )
        raise
    return pygame


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline pygame viewer for behavior_tree decision JSONL traces.")
    parser.add_argument("trace", nargs="?", default="", help="Decision trace JSONL. Defaults to config paths.sample_trace.")
    parser.add_argument("--config", default="", help="Optional YAML override for viewer layout, colors, field, and goals.")
    parser.add_argument("--map", dest="map_path", default="", help="Basemap image path.")
    parser.add_argument("--points-json", default="", help="Optional tools/maps map_plugin JSON/YAML with point coordinates.")
    parser.add_argument("--start-paused", action="store_true", help="Start playback paused.")
    parser.add_argument("--speed", type=float, default=0.0, help="Initial playback speed. 0 keeps YAML default.")
    parser.add_argument("--follow", action="store_true", help="Follow a growing trace file and update the view in real time.")
    parser.add_argument("--follow-poll", type=float, default=0.25, help="Follow mode poll interval in seconds (default: 0.25).")
    parser.add_argument("--follow-wait", type=float, default=20.0, help="Wait up to N seconds for first trace records in follow mode.")
    parser.add_argument(
        "--web-stream",
        dest="web_stream",
        action="store_true",
        help="Enable HTTP frame streaming from pygame window (overrides YAML).",
    )
    parser.add_argument(
        "--no-web-stream",
        dest="web_stream",
        action="store_false",
        help="Disable HTTP frame streaming (overrides YAML).",
    )
    parser.set_defaults(web_stream=None)
    parser.add_argument("--web-host", default="", help="HTTP stream bind host (overrides YAML web_stream.host).")
    parser.add_argument("--web-port", type=int, default=0, help="HTTP stream bind port (overrides YAML web_stream.port).")
    parser.add_argument("--web-fps", type=float, default=0.0, help="Max stream FPS (overrides YAML web_stream.fps).")
    parser.add_argument(
        "--web-jpeg-quality",
        type=int,
        default=0,
        help="JPEG quality in [1,100] (overrides YAML web_stream.jpeg_quality).",
    )
    parser.add_argument(
        "--control-file",
        default="",
        help="Path to match-control command JSONL (overrides YAML match_control.control_file).",
    )
    parser.add_argument(
        "--match-duration-sec",
        type=int,
        default=0,
        help="Match duration in seconds (overrides YAML match_control.duration_sec).",
    )
    parser.add_argument("--validate-only", action="store_true", help="Load trace/config and run offline consistency checks, then exit.")
    parser.add_argument("--smoke-test", action="store_true", help="Load config, trace, map, and pygame, draw one frame, then exit.")
    return parser.parse_args(argv)


def wait_for_follow_records(
    trace_path: Path,
    goal_names: dict[int, str],
    follow_poll: float,
    follow_wait: float,
    map_path: Path | None = None,
    pygame=None,
    config: dict | None = None,
    streamer=None,
    control_file: Path | None = None,
) -> tuple[list, int, int] | None:
    deadline = time.monotonic() + max(0.0, follow_wait)
    records: list = []
    bad_lines = 0
    follow_offset = 0
    poll_sec = max(0.05, float(follow_poll))
    next_poll = 0.0

    screen = None
    font = None
    small_font = None
    clock = None
    fps = 60
    if pygame is not None and isinstance(config, dict):
        window = config.get("window", {}) if isinstance(config.get("window"), dict) else {}
        width = int(window.get("width", 1500))
        height = int(window.get("height", 900))
        min_width = int(window.get("min_width", 960))
        min_height = int(window.get("min_height", 620))
        fps = int(window.get("fps", 60))
        screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        pygame.display.set_caption("LY Decision Visualization (Waiting Trace)")
        font = pygame.font.SysFont("DejaVu Sans", 24, bold=True)
        small_font = pygame.font.SysFont("DejaVu Sans", 16)
        clock = pygame.time.Clock()

    wait_map = None
    wait_map_size = (1, 1)
    if pygame is not None and map_path is not None and map_path.exists():
        try:
            wait_map = pygame.image.load(str(map_path)).convert_alpha()
            wait_map_size = wait_map.get_size()
        except Exception:
            wait_map = None

    while True:
        now = time.monotonic()
        if screen is not None:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return None
                if event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                    return None
                if event.type == pygame.KEYDOWN and event.key in (pygame.K_s, pygame.K_RETURN):
                    if control_file is not None:
                        try:
                            append_command(control_file, "start")
                        except OSError:
                            pass
                if event.type == pygame.KEYDOWN and event.key in (pygame.K_r,):
                    if control_file is not None:
                        try:
                            append_command(control_file, "reset")
                        except OSError:
                            pass
                if event.type == pygame.VIDEORESIZE:
                    new_w = max(min_width, event.w)
                    new_h = max(min_height, event.h)
                    screen = pygame.display.set_mode((new_w, new_h), pygame.RESIZABLE)

        if now >= next_poll:
            next_poll = now + poll_sec
            if trace_path.exists():
                records, bad_lines, follow_offset = load_trace_incremental(
                    trace_path,
                    goal_names,
                    start_offset=0,
                    start_index=0,
                )
            if records:
                return records, bad_lines, follow_offset

        if now >= deadline:
            raise TimeoutError(f"no trace records loaded from {trace_path} within {follow_wait:.1f}s")

        if screen is None:
            time.sleep(min(0.2, poll_sec))
            continue

        remaining = max(0.0, deadline - now)
        screen.fill(pygame.Color("#101216"))
        if wait_map is not None:
            area = pygame.Rect(18, 18, max(200, screen.get_width() - 36), max(120, screen.get_height() - 120))
            src_w, src_h = wait_map_size
            scale = min(area.width / src_w, area.height / src_h)
            draw_w = int(src_w * scale)
            draw_h = int(src_h * scale)
            draw_rect = pygame.Rect(
                area.x + (area.width - draw_w) // 2,
                area.y + (area.height - draw_h) // 2,
                draw_w,
                draw_h,
            )
            scaled = pygame.transform.smoothscale(wait_map, (draw_w, draw_h))
            screen.blit(scaled, draw_rect)
            pygame.draw.rect(screen, pygame.Color("#3a424d"), draw_rect, 1, border_radius=4)

        title = font.render("Waiting for decision trace...", True, pygame.Color("#edf2f7"))
        hint = small_font.render(f"trace: {trace_path}", True, pygame.Color("#a8b0ba"))
        remain = small_font.render(f"remaining: {remaining:.1f}s", True, pygame.Color("#a8b0ba"))
        cancel = small_font.render("Press ESC or close window to cancel.", True, pygame.Color("#a8b0ba"))
        start_hint = None
        if control_file is not None:
            start_hint = small_font.render("Press S or Enter to start offline match gate.", True, pygame.Color("#f5c542"))
        cx = screen.get_width() // 2
        screen.blit(title, (cx - title.get_width() // 2, screen.get_height() // 2 - 80))
        screen.blit(hint, (cx - hint.get_width() // 2, screen.get_height() // 2 - 24))
        screen.blit(remain, (cx - remain.get_width() // 2, screen.get_height() // 2 + 8))
        if start_hint is not None:
            screen.blit(start_hint, (cx - start_hint.get_width() // 2, screen.get_height() // 2 + 38))
            screen.blit(cancel, (cx - cancel.get_width() // 2, screen.get_height() // 2 + 68))
        else:
            screen.blit(cancel, (cx - cancel.get_width() // 2, screen.get_height() // 2 + 40))
        if streamer is not None:
            streamer.publish_surface(screen, pygame)
        pygame.display.flip()
        clock.tick(max(15, min(120, fps)))


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.follow_poll <= 0:
        print("--follow-poll must be > 0", file=sys.stderr)
        return 2

    config = load_config(resolve_path(args.config) if args.config else None)
    paths = config.get("paths", {}) if isinstance(config.get("paths"), dict) else {}
    web_cfg = config.get("web_stream", {}) if isinstance(config.get("web_stream"), dict) else {}
    match_cfg = config.get("match_control", {}) if isinstance(config.get("match_control"), dict) else {}

    web_stream_enabled = (
        bool(args.web_stream)
        if args.web_stream is not None
        else bool(web_cfg.get("enabled", True))
    )
    web_host = args.web_host.strip() if args.web_host.strip() else str(web_cfg.get("host", "0.0.0.0"))
    web_port = args.web_port if args.web_port > 0 else int(web_cfg.get("port", 9000))
    web_fps = args.web_fps if args.web_fps > 0 else float(web_cfg.get("fps", 12.0))
    web_jpeg_quality = (
        args.web_jpeg_quality if args.web_jpeg_quality > 0 else int(web_cfg.get("jpeg_quality", 80))
    )

    if web_port <= 0 or web_port > 65535:
        print("--web-port must be in [1, 65535]", file=sys.stderr)
        return 2
    if web_fps <= 0:
        print("--web-fps must be > 0", file=sys.stderr)
        return 2
    if web_jpeg_quality < 1 or web_jpeg_quality > 100:
        print("--web-jpeg-quality must be in [1, 100]", file=sys.stderr)
        return 2
    if args.match_duration_sec < 0:
        print("--match-duration-sec must be >= 0", file=sys.stderr)
        return 2

    if args.control_file.strip():
        match_cfg["control_file"] = args.control_file.strip()
    if args.match_duration_sec > 0:
        match_cfg["duration_sec"] = int(args.match_duration_sec)
    if match_cfg:
        config["match_control"] = match_cfg
    control_file = str(match_cfg.get("control_file", "")).strip()
    if control_file:
        control_file = str(Path(control_file).expanduser().resolve())
    control_file_path = Path(control_file) if control_file else None
    control_step_sec = int(match_cfg.get("rewind_step_sec", 10) or 10)

    trace_path = resolve_path(args.trace or paths.get("sample_trace", "src/decision_viz/sample/sample_trace.jsonl"))
    map_path = resolve_path(args.map_path or paths.get("default_map", "tools/maps/basemaps/buff_map_field.png"))
    if not args.follow and not trace_path.exists():
        print(f"trace file not found: {trace_path}", file=sys.stderr)
        return 2
    if not map_path.exists():
        print(f"map image not found: {map_path}", file=sys.stderr)
        return 2

    goals = goals_by_id(config)
    if args.points_json:
        goals.update(load_plugin_points(resolve_path(args.points_json)))
    goal_names = {goal_id: str(goal.get("name", f"Goal{goal_id}")) for goal_id, goal in goals.items()}

    records: list = []
    bad_lines = 0
    follow_offset = 0
    pygame = None
    pygame_initialized = False
    streamer = None

    if web_stream_enabled and not args.validate_only:
        try:
            from .web_stream import DecisionVizWebStream

            streamer = DecisionVizWebStream(
                host=web_host,
                port=web_port,
                fps=web_fps,
                jpeg_quality=web_jpeg_quality,
                control_file=control_file,
                default_step_sec=control_step_sec,
            )
            streamer.start()
            if web_host == "0.0.0.0":
                print(f"web stream: http://127.0.0.1:{web_port}/ (LAN: http://<your-ip>:{web_port}/)")
            else:
                print(f"web stream: http://{web_host}:{web_port}/")
        except Exception as exc:
            print(f"web stream disabled: {exc}", file=sys.stderr)
            streamer = None

    if args.follow:
        show_wait_window = not args.validate_only
        if show_wait_window:
            pygame = import_pygame()
            pygame.init()
            pygame_initialized = True

        while True:
            try:
                follow_result = wait_for_follow_records(
                    trace_path=trace_path,
                    goal_names=goal_names,
                    follow_poll=args.follow_poll,
                    follow_wait=args.follow_wait,
                    map_path=map_path,
                    pygame=pygame if show_wait_window else None,
                    config=config if show_wait_window else None,
                    streamer=streamer,
                    control_file=control_file_path,
                )
            except (OSError, TimeoutError) as exc:
                if pygame_initialized:
                    pygame.quit()
                if streamer is not None:
                    streamer.stop()
                print(str(exc), file=sys.stderr)
                return 2
            if follow_result is None:
                if pygame_initialized:
                    pygame.quit()
                if streamer is not None:
                    streamer.stop()
                return 130
            records, bad_lines, follow_offset = follow_result
            break
    else:
        try:
            records, bad_lines = load_trace(trace_path, goal_names)
        except (OSError, ValueError) as exc:
            print(str(exc), file=sys.stderr)
            return 2

    validation_issues = validate_records(records, config, bad_lines)
    if args.validate_only:
        print(format_validation(records, validation_issues))
        if pygame_initialized:
            pygame.quit()
        return 2 if any(issue.severity == "error" for issue in validation_issues) else 0

    if pygame is None:
        pygame = import_pygame()
        pygame.init()
        pygame_initialized = True

    try:
        viewer = Viewer(
            pygame=pygame,
            records=records,
            changes=build_changes(records),
            config=config,
            goals=goals,
            map_path=map_path,
            bad_lines=bad_lines,
            start_paused=True if args.start_paused else None,
            speed=args.speed if args.speed > 0 else None,
            trace_path=trace_path if args.follow else None,
            goal_names=goal_names if args.follow else None,
            follow=args.follow,
            follow_poll_sec=args.follow_poll,
            follow_offset=follow_offset,
            streamer=streamer,
        )
        if args.smoke_test:
            viewer.draw()
            if streamer is not None:
                streamer.publish_surface(viewer.screen, pygame)
            pygame.display.flip()
            return 0
        viewer.run()
    finally:
        if streamer is not None:
            streamer.stop()
        if pygame_initialized:
            pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
