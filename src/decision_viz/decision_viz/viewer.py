from __future__ import annotations

import bisect
import math
import sys
import time
from pathlib import Path
from typing import Any

from .control_bus import append_command, read_commands
from .model import TraceRecord, UnitRecord
from .trace import as_dict, as_list, build_changes, load_trace_incremental, parse_position


DEFAULT_FIELD_CM = (2800, 1500)


def fit_rect(pg: Any, src_size: tuple[int, int], dst_rect: Any) -> Any:
    src_w, src_h = src_size
    scale = min(dst_rect.width / src_w, dst_rect.height / src_h)
    width = int(src_w * scale)
    height = int(src_h * scale)
    return pg.Rect(
        dst_rect.x + (dst_rect.width - width) // 2,
        dst_rect.y + (dst_rect.height - height) // 2,
        width,
        height,
    )


def color(pg: Any, colors: dict[str, str], name: str, fallback: str) -> Any:
    return pg.Color(colors.get(name, fallback))


def record_position(record: TraceRecord, goals: dict[int, dict[str, Any]]) -> tuple[float, float] | None:
    if record.output.kind == "relative_target_bridge":
        return None
    goal = goals.get(record.goal_base_id)
    if goal:
        pos = goal.get(record.goal_side)
        if pos is not None and pos != (0, 0):
            return pos
    if record.output.goal_pos_cm is not None and record.output.uses_goal_pos:
        return record.output.goal_pos_cm
    if record.output.goal_pos_cm is not None:
        return record.output.goal_pos_cm
    return None


class Viewer:
    def __init__(
        self,
        pygame: Any,
        records: list[TraceRecord],
        changes: list[dict[str, Any]],
        config: dict[str, Any],
        goals: dict[int, dict[str, Any]],
        map_path: Path,
        bad_lines: int,
        start_paused: bool | None,
        speed: float | None,
        trace_path: Path | None = None,
        goal_names: dict[int, str] | None = None,
        follow: bool = False,
        follow_poll_sec: float = 0.25,
        follow_offset: int = 0,
        streamer: Any | None = None,
    ) -> None:
        self.pg = pygame
        self.records = records
        self.changes = changes
        self.config = config
        self.goals = goals
        self.map_path = map_path
        self.bad_lines = bad_lines
        self.trace_path = trace_path
        self.goal_names = goal_names or {}
        self.follow = bool(follow and trace_path is not None)
        self.follow_poll_sec = max(0.05, float(follow_poll_sec))
        self.follow_offset = max(0, int(follow_offset))
        self.last_follow_poll = time.perf_counter()
        self.streamer = streamer

        window = as_dict(config.get("window"))
        self.width = int(window.get("width", 1500))
        self.height = int(window.get("height", 900))
        self.min_width = int(window.get("min_width", 960))
        self.min_height = int(window.get("min_height", 620))
        self.panel_w = int(window.get("panel_width", 390))
        self.timeline_h = int(window.get("timeline_height", 92))
        self.playing = not (window.get("start_paused", False) if start_paused is None else start_paused)
        self.playback_speed = float(window.get("playback_speed", 1.0) if speed is None else speed)
        self.show_labels = bool(window.get("show_point_labels", False))

        self.layers = as_dict(config.get("layers"))
        self.timeline_config = as_dict(config.get("timeline"))
        self.colors_raw = as_dict(config.get("colors"))
        self.unit_styles = as_dict(config.get("unit_styles"))
        self.times = [record.t for record in records]
        self.current_index = 0
        self.current_time = self.times[0]
        self.match_control = as_dict(config.get("match_control"))
        self.match_control_enabled = bool(self.match_control.get("enabled", False))
        self.match_duration_sec = max(1, int(self.match_control.get("duration_sec", 420)))
        self.rewind_step_sec = max(1, int(self.match_control.get("rewind_step_sec", 10)))
        self.forward_step_sec = max(1, int(self.match_control.get("forward_step_sec", 10)))
        control_file = str(self.match_control.get("control_file", "")).strip()
        self.control_path: Path | None = None
        if control_file:
            self.control_path = Path(control_file).expanduser().resolve()
        self.control_buttons: dict[str, Any] = {}
        self.last_control_status = "idle"
        initial_time_left = self.records[self.current_index].time_left
        if initial_time_left <= 0:
            initial_time_left = self.match_duration_sec
        self.match_time_left_sec = float(max(0, min(self.match_duration_sec, int(initial_time_left))))
        self.match_started = False
        self.match_running = False
        self.last_trace_time_left = int(round(self.match_time_left_sec))
        self.last_control_poll = time.perf_counter()
        self.control_poll_sec = 0.08
        self.control_read_offset = 0
        if self.control_path is not None and self.control_path.exists():
            try:
                self.control_read_offset = int(self.control_path.stat().st_size)
            except OSError:
                self.control_read_offset = 0

        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("LY Decision Visualization")
        try:
            driver = pygame.display.get_driver()
            if str(driver).strip().lower() == "offscreen":
                print(
                    "warning: SDL video driver is offscreen; pygame window may be invisible (WSL GUI/X11/Wayland issue).",
                    file=sys.stderr,
                )
        except Exception:
            pass
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("DejaVu Sans", 16)
        self.small_font = pygame.font.SysFont("DejaVu Sans", 13)
        self.title_font = pygame.font.SysFont("DejaVu Sans", 22, bold=True)
        self.mono_font = pygame.font.SysFont("DejaVu Sans Mono", 14)

        self.palette = {
            "bg": color(pygame, self.colors_raw, "bg", "#101216"),
            "panel": color(pygame, self.colors_raw, "panel", "#1b2026"),
            "panel2": color(pygame, self.colors_raw, "panel2", "#252b33"),
            "text": color(pygame, self.colors_raw, "text", "#edf2f7"),
            "muted": color(pygame, self.colors_raw, "muted", "#a8b0ba"),
            "line": color(pygame, self.colors_raw, "line", "#3a424d"),
            "accent": color(pygame, self.colors_raw, "accent", "#f5c542"),
            "friend": color(pygame, self.colors_raw, "friend", "#4cc38a"),
            "enemy": color(pygame, self.colors_raw, "enemy", "#e85d5d"),
            "red": color(pygame, self.colors_raw, "red", "#e85d5d"),
            "blue": color(pygame, self.colors_raw, "blue", "#5d8ee8"),
            "black": color(pygame, self.colors_raw, "black", "#000000"),
            "white": color(pygame, self.colors_raw, "white", "#ffffff"),
            "neutral": color(pygame, self.colors_raw, "neutral", "#4fb3d9"),
        }

        self.map_image = pygame.image.load(str(map_path)).convert_alpha()
        self.map_size = self.map_image.get_size()
        self.scaled_map = None
        self.cached_map_key: tuple[int, int, int, int] | None = None

    def run(self) -> None:
        last = time.perf_counter()
        running = True
        while running:
            now = time.perf_counter()
            dt = now - last
            last = now
            for event in self.pg.event.get():
                if event.type == self.pg.QUIT:
                    running = False
                elif event.type == self.pg.VIDEORESIZE:
                    self.width = max(self.min_width, event.w)
                    self.height = max(self.min_height, event.h)
                    self.screen = self.pg.display.set_mode((self.width, self.height), self.pg.RESIZABLE)
                    self.cached_map_key = None
                elif event.type == self.pg.KEYDOWN:
                    self.handle_key(event.key)
                elif event.type == self.pg.MOUSEBUTTONDOWN:
                    self.handle_mouse(event)

            if self.follow and (now - self.last_control_poll) >= self.control_poll_sec:
                self.poll_control_commands()
                self.last_control_poll = now

            if self.follow and (now - self.last_follow_poll) >= self.follow_poll_sec:
                self.poll_trace_updates()
                self.last_follow_poll = now

            self.tick_match_clock(dt)

            if self.playing and len(self.records) > 1:
                self.current_time += dt * self.playback_speed
                if self.current_time >= self.times[-1]:
                    # In follow mode keep tailing latest records instead of auto-pausing.
                    if self.follow:
                        self.current_time = self.times[-1]
                    else:
                        self.current_time = self.times[-1]
                        self.playing = False
                self.current_index = min(
                    len(self.records) - 1,
                    max(0, bisect.bisect_right(self.times, self.current_time) - 1),
                )

            self.draw()
            if self.streamer is not None:
                self.streamer.publish_surface(self.screen, self.pg)
            self.pg.display.flip()
            self.clock.tick(int(as_dict(self.config.get("window")).get("fps", 60)))

    def handle_key(self, key: int) -> None:
        pg = self.pg
        jump = int(self.timeline_config.get("jump_step", 25))
        if key in (pg.K_ESCAPE, pg.K_q):
            pg.event.post(pg.event.Event(pg.QUIT))
        elif key == pg.K_SPACE:
            self.playing = not self.playing
            self.current_time = self.records[self.current_index].t
        elif key in (pg.K_RIGHT, pg.K_PERIOD):
            self.seek_index(self.current_index + 1)
        elif key in (pg.K_LEFT, pg.K_COMMA):
            self.seek_index(self.current_index - 1)
        elif key == pg.K_PAGEUP:
            self.seek_index(self.current_index + jump)
        elif key == pg.K_PAGEDOWN:
            self.seek_index(self.current_index - jump)
        elif key == pg.K_HOME:
            self.seek_index(0)
        elif key == pg.K_END:
            self.seek_index(len(self.records) - 1)
        elif key in (pg.K_EQUALS, pg.K_PLUS, pg.K_KP_PLUS):
            self.playback_speed = min(16.0, self.playback_speed * 1.5)
        elif key in (pg.K_MINUS, pg.K_KP_MINUS):
            self.playback_speed = max(0.1, self.playback_speed / 1.5)
        elif key == pg.K_l:
            self.show_labels = not self.show_labels
        elif key == pg.K_s:
            self.send_match_command("start")
        elif key == pg.K_p:
            self.send_match_command("pause")
        elif key == pg.K_r:
            self.send_match_command("reset")
        elif key == pg.K_LEFTBRACKET:
            self.send_match_command("rewind", {"seconds": self.rewind_step_sec})
        elif key == pg.K_RIGHTBRACKET:
            self.send_match_command("forward", {"seconds": self.forward_step_sec})

    def handle_mouse(self, event: Any) -> None:
        if event.button != 1:
            return
        for command, rect in self.control_buttons.items():
            if rect.collidepoint(event.pos):
                if command == "rewind":
                    self.send_match_command(command, {"seconds": self.rewind_step_sec})
                elif command == "forward":
                    self.send_match_command(command, {"seconds": self.forward_step_sec})
                else:
                    self.send_match_command(command)
                return
        track = self.timeline_rect()
        if track.collidepoint(event.pos):
            ratio = (event.pos[0] - track.x) / max(1, track.width)
            self.seek_index(round(ratio * (len(self.records) - 1)))

    def send_match_command(self, command: str, payload: dict[str, Any] | None = None) -> None:
        if not self.match_control_enabled or self.control_path is None or not self.follow:
            return
        try:
            append_command(self.control_path, command, payload)
            self.apply_local_match_command(command, payload)
            self.last_control_status = f"cmd={command}"
        except OSError:
            self.last_control_status = f"cmd={command} failed"

    def apply_local_match_command(self, command: str, payload: dict[str, Any] | None = None) -> None:
        cmd = str(command).strip().lower()
        body = payload or {}
        if cmd == "start":
            if self.match_time_left_sec <= 0.0:
                self.match_time_left_sec = float(self.match_duration_sec)
            self.match_started = True
            self.match_running = True
            return
        if cmd == "pause":
            self.match_running = False
            return
        if cmd == "reset":
            self.match_time_left_sec = float(self.match_duration_sec)
            self.match_started = False
            self.match_running = False
            return
        if cmd in {"rewind", "forward", "set_time_left"}:
            try:
                sec = float(body.get("seconds", 0.0))
            except (TypeError, ValueError):
                return
            if not math.isfinite(sec):
                return
            if cmd == "rewind":
                self.match_time_left_sec += max(0.0, sec)
            elif cmd == "forward":
                self.match_time_left_sec -= max(0.0, sec)
            else:
                self.match_time_left_sec = sec
            self.match_time_left_sec = max(0.0, min(float(self.match_duration_sec), self.match_time_left_sec))
            if self.match_time_left_sec <= 0.0:
                self.match_running = False
            return

    def poll_control_commands(self) -> None:
        if not self.match_control_enabled or self.control_path is None or not self.follow:
            return
        commands, new_offset = read_commands(self.control_path, self.control_read_offset)
        self.control_read_offset = new_offset
        for payload in commands:
            command = str(payload.get("command", "")).strip().lower()
            if not command:
                continue
            self.apply_local_match_command(command, payload)
            self.last_control_status = f"cmd={command}"

    def tick_match_clock(self, dt: float) -> None:
        if not self.match_control_enabled or not self.follow:
            return
        if not self.match_started or not self.match_running:
            return
        if self.match_time_left_sec <= 0.0:
            self.match_running = False
            return
        self.match_time_left_sec = max(0.0, self.match_time_left_sec - max(0.0, float(dt)))
        if self.match_time_left_sec <= 0.0:
            self.match_running = False

    def seek_index(self, index: int) -> None:
        self.current_index = max(0, min(len(self.records) - 1, index))
        self.current_time = self.records[self.current_index].t

    def poll_trace_updates(self) -> None:
        if self.trace_path is None:
            return
        if not self.trace_path.exists():
            return
        try:
            new_records, bad_lines, new_offset = load_trace_incremental(
                self.trace_path,
                self.goal_names,
                self.follow_offset,
                len(self.records),
            )
        except OSError:
            return
        self.follow_offset = new_offset
        self.bad_lines += bad_lines
        if not new_records:
            return

        at_tail = self.current_index >= (len(self.records) - 1)
        self.records.extend(new_records)
        self.times.extend(record.t for record in new_records)
        self.changes = build_changes(self.records)
        latest_time_left = int(new_records[-1].time_left)
        latest_time_left = max(0, min(self.match_duration_sec, latest_time_left))
        if latest_time_left != self.last_trace_time_left:
            self.match_time_left_sec = float(latest_time_left)
            self.last_trace_time_left = latest_time_left
            if latest_time_left < self.match_duration_sec:
                self.match_started = True
            if latest_time_left <= 0:
                self.match_running = False

        if at_tail:
            self.current_index = len(self.records) - 1
            self.current_time = self.records[self.current_index].t

    def map_area_rect(self) -> Any:
        return self.pg.Rect(18, 18, self.width - self.panel_w - 36, self.height - self.timeline_h - 32)

    def panel_rect(self) -> Any:
        return self.pg.Rect(self.width - self.panel_w, 0, self.panel_w, self.height - self.timeline_h)

    def timeline_rect(self) -> Any:
        return self.pg.Rect(28, self.height - 52, self.width - 56, 16)

    def draw(self) -> None:
        self.screen.fill(self.palette["bg"])
        self.draw_map()
        self.draw_panel()
        self.draw_timeline()

    def draw_map(self) -> None:
        pg = self.pg
        area = self.map_area_rect()
        pg.draw.rect(self.screen, self.palette["panel"], area, border_radius=8)
        image_rect = fit_rect(pg, self.map_size, area.inflate(-18, -18))
        rect_key = (image_rect.x, image_rect.y, image_rect.width, image_rect.height)
        if self.scaled_map is None or self.cached_map_key != rect_key:
            self.scaled_map = pg.transform.smoothscale(self.map_image, (image_rect.width, image_rect.height))
            self.cached_map_key = rect_key
        self.screen.blit(self.scaled_map, image_rect)
        pg.draw.rect(self.screen, self.palette["line"], image_rect, 1, border_radius=4)

        if self.layers.get("terrain", True):
            self.draw_terrain(image_rect)
        if self.layers.get("structures", True):
            self.draw_structures(image_rect)
        if self.layers.get("grid", True):
            self.draw_grid(image_rect)
        if self.layers.get("all_goals", True):
            self.draw_all_goals(image_rect)
        if self.layers.get("goal_path", True):
            self.draw_path(image_rect)
        if self.layers.get("units", True):
            self.draw_units(image_rect)
        if self.layers.get("current_goal", True):
            self.draw_current_goal(image_rect)

    def field_size(self) -> tuple[int, int]:
        record = self.records[self.current_index]
        field = as_dict(record.raw.get("field_cm")) or as_dict(self.config.get("field_cm"))
        return (int(field.get("width", DEFAULT_FIELD_CM[0])), int(field.get("height", DEFAULT_FIELD_CM[1])))

    def field_to_screen(self, pos: tuple[float, float], image_rect: Any) -> tuple[int, int]:
        field_w, field_h = self.field_size()
        x = max(0.0, min(float(field_w), pos[0]))
        y = max(0.0, min(float(field_h), pos[1]))
        sx = image_rect.x + x / field_w * image_rect.width
        sy = image_rect.y + (1.0 - y / field_h) * image_rect.height
        return (round(sx), round(sy))

    def draw_grid(self, image_rect: Any) -> None:
        pg = self.pg
        field_w, field_h = self.field_size()
        overlay = pg.Surface((image_rect.width, image_rect.height), pg.SRCALPHA)
        grid_color = pg.Color(255, 255, 255, 34)
        for x_cm in range(0, field_w + 1, 400):
            x = round(x_cm / field_w * image_rect.width)
            pg.draw.line(overlay, grid_color, (x, 0), (x, image_rect.height), 1)
        for y_cm in range(0, field_h + 1, 300):
            y = round((1.0 - y_cm / field_h) * image_rect.height)
            pg.draw.line(overlay, grid_color, (0, y), (image_rect.width, y), 1)
        self.screen.blit(overlay, image_rect.topleft)

    def draw_terrain(self, image_rect: Any) -> None:
        terrain = as_dict(self.config.get("terrain"))
        if not terrain.get("enabled", True):
            return
        levels = as_dict(terrain.get("levels"))
        overlay = self.pg.Surface((image_rect.width, image_rect.height), self.pg.SRCALPHA)
        labels: list[tuple[str, int, int]] = []
        for zone in as_list(terrain.get("zones")):
            if not isinstance(zone, dict):
                continue
            points = self.terrain_zone_points(zone, image_rect)
            if len(points) < 3:
                continue
            level = as_dict(levels.get(str(zone.get("level", "ground"))))
            fill = self.pg.Color(level.get("color", "#4fb3d9"))
            fill.a = int(level.get("alpha", 46))
            outline = self.pg.Color(level.get("outline", level.get("color", "#4fb3d9")))
            outline.a = int(level.get("outline_alpha", 110))
            self.pg.draw.polygon(overlay, fill, points)
            self.pg.draw.lines(overlay, outline, True, points, 2)
            if terrain.get("show_labels", False) or self.show_labels:
                cx = round(sum(point[0] for point in points) / len(points)) + image_rect.x
                cy = round(sum(point[1] for point in points) / len(points)) + image_rect.y
                label = f"{zone.get('name', 'terrain')} {level.get('label', zone.get('level', ''))}"
                labels.append((str(label), cx + 4, cy - 10))
        self.screen.blit(overlay, image_rect.topleft)
        for label, x, y in labels:
            self.draw_label(label, x, y, image_rect)

    def draw_structures(self, image_rect: Any) -> None:
        structures = as_dict(self.config.get("structures"))
        if not structures.get("enabled", True):
            return

        styles = as_dict(structures.get("styles"))
        show_labels = bool(structures.get("show_labels", False) or self.show_labels)
        overlay = self.pg.Surface((image_rect.width, image_rect.height), self.pg.SRCALPHA)
        labels: list[tuple[str, int, int]] = []

        for item in as_list(structures.get("items")):
            if not isinstance(item, dict):
                continue
            kind = str(item.get("kind", "wall"))
            style = as_dict(styles.get(kind))
            fill = self.pg.Color(style.get("fill", "#708090"))
            fill.a = int(style.get("alpha", 44))
            outline = self.pg.Color(style.get("outline", style.get("fill", "#708090")))
            outline.a = int(style.get("outline_alpha", 130))
            label_name = str(item.get("name", kind))
            shape = str(item.get("shape", "polygon"))

            if shape == "circle":
                center = parse_position(item.get("center"))
                if center is None:
                    continue
                radius_cm = float(item.get("radius", 0.0))
                if not math.isfinite(radius_cm) or radius_cm <= 0.0:
                    continue
                sx, sy = self.field_to_screen(center, image_rect)
                field_w, field_h = self.field_size()
                px_per_cm_x = image_rect.width / max(1.0, float(field_w))
                px_per_cm_y = image_rect.height / max(1.0, float(field_h))
                radius_px = max(2, int(round(radius_cm * min(px_per_cm_x, px_per_cm_y))))
                local_center = (sx - image_rect.x, sy - image_rect.y)
                self.pg.draw.circle(overlay, fill, local_center, radius_px)
                self.pg.draw.circle(overlay, outline, local_center, radius_px, 2)
                if show_labels:
                    labels.append((label_name, sx + radius_px + 6, sy - 8))
                continue

            points = self.structure_points(item, image_rect)
            if len(points) < 3:
                continue
            self.pg.draw.polygon(overlay, fill, points)
            self.pg.draw.lines(overlay, outline, True, points, 2)
            if show_labels:
                cx = round(sum(point[0] for point in points) / len(points)) + image_rect.x
                cy = round(sum(point[1] for point in points) / len(points)) + image_rect.y
                labels.append((label_name, cx + 4, cy - 10))

        self.screen.blit(overlay, image_rect.topleft)
        for label, x, y in labels:
            self.draw_label(label, x, y, image_rect)

    def structure_points(self, item: dict[str, Any], image_rect: Any) -> list[tuple[int, int]]:
        raw_points = as_list(item.get("polygon"))
        if not raw_points and isinstance(item.get("rect"), (list, tuple)) and len(item["rect"]) >= 4:
            x, y, w, h = item["rect"][:4]
            raw_points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]

        points: list[tuple[int, int]] = []
        for raw in raw_points:
            pos = parse_position(raw)
            if pos is None:
                continue
            sx, sy = self.field_to_screen(pos, image_rect)
            points.append((sx - image_rect.x, sy - image_rect.y))
        return points

    def terrain_zone_points(self, zone: dict[str, Any], image_rect: Any) -> list[tuple[int, int]]:
        raw_points = as_list(zone.get("polygon"))
        if not raw_points and isinstance(zone.get("rect"), (list, tuple)) and len(zone["rect"]) >= 4:
            x, y, w, h = zone["rect"][:4]
            raw_points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        points: list[tuple[int, int]] = []
        for raw in raw_points:
            pos = parse_position(raw)
            if pos is None:
                continue
            sx, sy = self.field_to_screen(pos, image_rect)
            points.append((sx - image_rect.x, sy - image_rect.y))
        return points

    def draw_all_goals(self, image_rect: Any) -> None:
        pg = self.pg
        for goal_id, goal in self.goals.items():
            for side in ("red", "blue"):
                pos = goal.get(side)
                if pos is None or pos == (0, 0):
                    continue
                sx, sy = self.field_to_screen(pos, image_rect)
                pg.draw.circle(self.screen, self.palette["black"], (sx, sy), 5)
                pg.draw.circle(self.screen, self.palette[side], (sx, sy), 4)
                if self.show_labels:
                    self.draw_text(
                        f"{goal_id}:{goal.get('name', '')}",
                        sx + 6,
                        sy - 7,
                        self.small_font,
                        self.palette["text"],
                        160,
                    )

    def path_points(self) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        last_goal: tuple[int, str, tuple[float, float] | None] | None = None
        limit = int(self.timeline_config.get("path_history_limit", 500))
        for record in self.records[max(0, self.current_index - limit) : self.current_index + 1]:
            key = record.output.route_key
            if key == last_goal:
                continue
            pos = record_position(record, self.goals)
            if pos is None:
                continue
            points.append(pos)
            last_goal = key
        return points

    def draw_path(self, image_rect: Any) -> None:
        pg = self.pg
        points = [self.field_to_screen(pos, image_rect) for pos in self.path_points()]
        if len(points) < 2:
            return
        pg.draw.lines(self.screen, self.palette["accent"], False, points, 3)
        for point in points[:-1]:
            pg.draw.circle(self.screen, self.palette["accent"], point, 4)

    def draw_current_goal(self, image_rect: Any) -> None:
        pg = self.pg
        record = self.records[self.current_index]
        pos = record_position(record, self.goals)
        if pos is None:
            return
        sx, sy = self.field_to_screen(pos, image_rect)
        side_color = self.palette[record.output.goal_side] if record.output.goal_side in ("red", "blue") else self.palette["accent"]
        pulse = 2 + int((time.perf_counter() * 4) % 4)
        pg.draw.circle(self.screen, self.palette["black"], (sx, sy), 15 + pulse)
        pg.draw.circle(self.screen, side_color, (sx, sy), 12 + pulse)
        pg.draw.circle(self.screen, self.palette["white"], (sx, sy), 5)
        label = f"{record.output.goal_name} id={record.output.goal_id}"
        self.draw_label(label, sx + 14, sy - 30, image_rect)

    def draw_units(self, image_rect: Any) -> None:
        for unit in self.records[self.current_index].units:
            self.draw_unit(unit, image_rect)

    def draw_unit(self, unit: UnitRecord, image_rect: Any) -> None:
        if unit.position_cm is None:
            return
        if not (math.isfinite(unit.position_cm[0]) and math.isfinite(unit.position_cm[1])):
            return
        pg = self.pg
        sx, sy = self.field_to_screen(unit.position_cm, image_rect)
        type_styles = as_dict(self.unit_styles.get("types"))
        style = as_dict(type_styles.get(unit.type_name, type_styles.get("default", {})))
        side_style = as_dict(self.unit_styles.get(unit.side, {}))
        radius = int(style.get("radius", 7))
        label = str(style.get("label", unit.type_name[:1] or "?"))
        fill = pg.Color(side_style.get("color", self.colors_raw.get(unit.side, "#4fb3d9")))
        outline = pg.Color(side_style.get("outline", "#000000"))
        pg.draw.circle(self.screen, outline, (sx, sy), radius + 3)
        pg.draw.circle(self.screen, fill, (sx, sy), radius + 1)
        text = self.small_font.render(label, True, self.palette["black"])
        self.screen.blit(text, text.get_rect(center=(sx, sy)))
        if self.layers.get("unit_health_bars", True) and unit.health_ratio is not None:
            self.draw_health_bar(sx - 17, sy + radius + 6, 34, 5, unit.health_ratio)
        if self.show_labels:
            suffix = f" {unit.hp}/{unit.max_hp}" if unit.max_hp else ""
            self.draw_label(f"{unit.side}:{unit.type_name}{suffix}", sx + 10, sy + 8, image_rect)

    def draw_health_bar(self, x: int, y: int, width: int, height: int, ratio: float) -> None:
        pg = self.pg
        ratio = max(0.0, min(1.0, ratio))
        fill = self.palette["friend"] if ratio >= 0.45 else self.palette["accent"] if ratio >= 0.2 else self.palette["enemy"]
        rect = pg.Rect(x, y, width, height)
        pg.draw.rect(self.screen, self.palette["black"], rect.inflate(2, 2), border_radius=3)
        pg.draw.rect(self.screen, self.palette["panel2"], rect, border_radius=3)
        if ratio > 0:
            pg.draw.rect(self.screen, fill, pg.Rect(x, y, max(1, round(width * ratio)), height), border_radius=3)

    def draw_label(self, label: str, x: int, y: int, bounds: Any) -> None:
        pg = self.pg
        surface = self.small_font.render(label, True, self.palette["text"])
        rect = surface.get_rect()
        rect.topleft = (
            min(max(bounds.x + 8, x), bounds.right - rect.width - 14),
            min(max(bounds.y + 8, y), bounds.bottom - rect.height - 10),
        )
        bg = rect.inflate(12, 8)
        pg.draw.rect(self.screen, self.palette["panel2"], bg, border_radius=5)
        self.screen.blit(surface, rect)

    def draw_panel(self) -> None:
        pg = self.pg
        rect = self.panel_rect()
        pg.draw.rect(self.screen, self.palette["panel"], rect)
        pg.draw.line(self.screen, self.palette["line"], rect.topleft, rect.bottomleft, 1)
        record = self.records[self.current_index]
        x = rect.x + 18
        y = rect.y + 18
        y = self.draw_text("Decision Viz", x, y, self.title_font, self.palette["text"], rect.width - 36)
        y += 6
        state = "PLAY" if self.playing else "PAUSE"
        y = self.draw_text(
            f"{state} {self.current_index + 1}/{len(self.records)} t={record.t:.2f}s x{self.playback_speed:.2g}",
            x,
            y,
            self.mono_font,
            self.palette["muted"],
            rect.width - 36,
        )
        y = self.draw_match_controls(x, y + 6, rect.width - 36, record)
        if self.bad_lines:
            y = self.draw_text(f"Skipped bad lines: {self.bad_lines}", x, y, self.small_font, self.palette["enemy"], rect.width - 36)
        y += 10
        y = self.draw_section(x, y, "Decision", [
            ("Profile", str(record.raw.get("competition_profile", "-"))),
            ("Team", record.team),
            ("Strategy", record.strategy),
            ("Aim", record.aim),
            ("Target", record.target),
        ], rect.width - 36)
        y = self.draw_section(x, y, "Decision Output", [
            ("Kind", record.output.kind),
            ("Goal", f"{record.output.goal_name} ({record.output.goal_id})"),
            ("Side", record.output.goal_side),
            ("Speed", str(record.output.speed_level)),
            ("Position", self.format_position(record_position(record, self.goals))),
            ("Topic", record.output.topic_text()),
            ("Publish", record.output.publish_text()),
        ], rect.width - 36)
        y = self.draw_section(x, y, "Posture", [
            ("Command", record.posture_command),
            ("State", record.posture_state),
            ("Current", record.posture_current),
            ("Desired", record.posture_desired),
            ("Pending", record.posture_pending),
            ("Reason", record.posture_reason),
        ], rect.width - 36)
        y = self.draw_section(x, y, "Referee", [
            ("HP", str(record.hp)),
            ("Ammo", str(record.ammo)),
            ("TimeLeft", str(record.time_left)),
            ("Outpost", self.outpost_text(record.raw)),
        ], rect.width - 36)
        y = self.draw_resource_bars(x, y, rect.width - 36, record)
        if self.layers.get("recent_changes", True):
            self.draw_recent_changes(x, y, rect)

    def draw_match_controls(self, x: int, y: int, max_width: int, record: TraceRecord) -> int:
        pg = self.pg
        y += 3
        self.draw_text("Match Clock", x, y, self.font, self.palette["accent"], max_width)
        y += 24
        controls_available = bool(self.match_control_enabled and self.control_path is not None and self.follow)
        if controls_available:
            remaining = max(0, min(self.match_duration_sec, int(math.ceil(self.match_time_left_sec))))
            state = "RUN" if self.match_running else ("READY" if self.match_started else "WAIT_START")
        else:
            remaining = record.time_left if record.time_left > 0 else self.match_duration_sec
            remaining = max(0, min(self.match_duration_sec, int(remaining)))
            state = "TRACE"
        summary = f"{state} {self.format_mmss(remaining)} / {self.format_mmss(self.match_duration_sec)}"
        y = self.draw_text(summary, x, y, self.mono_font, self.palette["text"], max_width)

        self.control_buttons = {}
        if controls_available:
            y += 8
            gap = 6
            labels = [
                ("start", "Start"),
                ("pause", "Pause"),
                ("rewind", f"+{self.rewind_step_sec}s"),
                ("forward", f"-{self.forward_step_sec}s"),
                ("reset", "Reset"),
            ]
            count = len(labels)
            button_w = max(54, (max_width - gap * (count - 1)) // count)
            button_h = 26
            for idx, (command, label) in enumerate(labels):
                bx = x + idx * (button_w + gap)
                rect = pg.Rect(bx, y, button_w, button_h)
                self.control_buttons[command] = rect
                self.draw_control_button(rect, label)
            y += button_h + 6
            y = self.draw_text(self.last_control_status, x, y, self.small_font, self.palette["muted"], max_width)
        else:
            y += 5
            y = self.draw_text("controls: disabled", x, y, self.small_font, self.palette["muted"], max_width)

        y += 8
        pg.draw.line(self.screen, self.palette["line"], (x, y), (x + max_width, y), 1)
        return y + 8

    def draw_control_button(self, rect: Any, label: str) -> None:
        pg = self.pg
        pg.draw.rect(self.screen, self.palette["panel2"], rect, border_radius=5)
        pg.draw.rect(self.screen, self.palette["line"], rect, 1, border_radius=5)
        txt = self.small_font.render(label, True, self.palette["text"])
        self.screen.blit(txt, txt.get_rect(center=rect.center))

    def draw_section(self, x: int, y: int, title: str, rows: list[tuple[str, str]], max_width: int) -> int:
        pg = self.pg
        y += 3
        self.draw_text(title, x, y, self.font, self.palette["accent"], max_width)
        y += 24
        for key, value in rows:
            key_surface = self.small_font.render(key, True, self.palette["muted"])
            self.screen.blit(key_surface, (x, y + 2))
            y = self.draw_text(value, x + 96, y, self.small_font, self.palette["text"], max_width - 96)
            y += 2
        y += 8
        pg.draw.line(self.screen, self.palette["line"], (x, y), (x + max_width, y), 1)
        return y + 8

    def draw_resource_bars(self, x: int, y: int, max_width: int, record: TraceRecord) -> int:
        referee = as_dict(record.raw.get("referee"))
        max_hp = max(record.hp, int(referee.get("self_max_hp", 400) or 400))
        ammo_max = int(as_dict(self.config.get("resources")).get("ammo_max", 50))
        self.draw_metric_bar(x + 96, y, max_width - 96, record.hp, max_hp)
        self.draw_metric_bar(x + 96, y + 8, max_width - 96, record.ammo, ammo_max)
        return y + 18

    def draw_metric_bar(self, x: int, y: int, width: int, value: int, maximum: int) -> None:
        if width <= 0 or maximum <= 0:
            return
        ratio = max(0.0, min(1.0, value / maximum))
        self.draw_health_bar(x, y, width, 5, ratio)

    def draw_recent_changes(self, x: int, y: int, rect: Any) -> None:
        self.draw_text("Recent Changes", x, y, self.font, self.palette["accent"], rect.width - 36)
        y += 24
        indexes = [item["index"] for item in self.changes]
        current = bisect.bisect_right(indexes, self.current_index) - 1
        visible = self.changes[max(0, current - 7) : current + 1]
        for item in visible:
            color_value = self.palette["text"] if item["index"] == self.current_index else self.palette["muted"]
            y = self.draw_text(f"{item['t']:.1f}s {item['text']}", x, y, self.small_font, color_value, rect.width - 36)
            y += 3
            if y > rect.bottom - 30:
                break

    def draw_timeline(self) -> None:
        pg = self.pg
        panel = pg.Rect(0, self.height - self.timeline_h, self.width, self.timeline_h)
        pg.draw.rect(self.screen, self.palette["panel"], panel)
        pg.draw.line(self.screen, self.palette["line"], panel.topleft, panel.topright, 1)
        track = self.timeline_rect()
        pg.draw.rect(self.screen, self.palette["panel2"], track, border_radius=8)
        ratio = self.current_index / max(1, len(self.records) - 1)
        pg.draw.rect(self.screen, self.palette["accent"], pg.Rect(track.x, track.y, round(track.width * ratio), track.height), border_radius=8)
        for item in self.changes:
            x = track.x + round(track.width * item["index"] / max(1, len(self.records) - 1))
            pg.draw.line(self.screen, self.palette["neutral"], (x, track.y - 6), (x, track.bottom + 6), 1)
        record = self.records[self.current_index]
        self.draw_text(self.map_path.name, track.x, track.y - 28, self.small_font, self.palette["muted"], track.width // 2)
        right = f"tick={record.tick} event={record.event} output={record.output.goal_name}:{record.output.goal_id}"
        text_w = self.small_font.size(right)[0]
        self.draw_text(right, track.right - text_w, track.y - 28, self.small_font, self.palette["muted"], text_w + 4)

    def draw_text(self, text: str, x: int, y: int, font: Any, draw_color: Any, max_width: int) -> int:
        if max_width <= 0:
            return y
        words = str(text).split() or [""]
        line = ""
        line_height = font.get_linesize()
        for word in words:
            candidate = word if not line else f"{line} {word}"
            if font.size(candidate)[0] <= max_width:
                line = candidate
                continue
            if line:
                self.screen.blit(font.render(line, True, draw_color), (x, y))
                y += line_height
            line = self.fit_word(word, font, max_width)
        if line:
            self.screen.blit(font.render(line, True, draw_color), (x, y))
            y += line_height
        return y

    @staticmethod
    def fit_word(word: str, font: Any, max_width: int) -> str:
        if font.size(word)[0] <= max_width:
            return word
        out = word
        while out and font.size(out + "...")[0] > max_width:
            out = out[:-1]
        return out + "..." if out else "..."

    @staticmethod
    def format_position(pos: tuple[float, float] | None) -> str:
        if pos is None:
            return "-"
        return f"{pos[0]:.0f}, {pos[1]:.0f} cm"

    @staticmethod
    def format_mmss(total_seconds: int) -> str:
        total = max(0, int(total_seconds))
        return f"{total // 60:02d}:{total % 60:02d}"

    @staticmethod
    def publish_text(raw: dict[str, Any]) -> str:
        navi = as_dict(raw.get("navi_goal"))
        allowed = navi.get("publish_allowed")
        enabled = navi.get("publish_enabled")
        if allowed is None and enabled is None:
            return "-"
        return f"allowed={allowed} enabled={enabled}"

    @staticmethod
    def outpost_text(raw: dict[str, Any]) -> str:
        referee = as_dict(raw.get("referee"))
        self_hp = referee.get("self_outpost_hp")
        enemy_hp = referee.get("enemy_outpost_hp")
        if self_hp is None and enemy_hp is None:
            return "-"
        return f"self={self_hp} enemy={enemy_hp}"
