# Decision Visualization Trace And Viewer

Updated: 2026-04-27

## Purpose

`decision_viz` is the maintained offline viewer for sentry decision behavior.
It replays JSONL rows written by `behavior_tree` and draws the current decision on a 2D field map.

This is for decision review and offline decision simulation. The viewer itself does not publish ROS topics.
In offline mode, it can send file-based control commands to `decision_viz.mock_inputs` for match clock control.

## Components

- Trace writer: `src/behavior_tree/src/DecisionTrace.cpp`
- Launch parameters: `decision_trace_enabled`, `decision_trace_file`, `decision_trace_every_n_ticks`
- Viewer package: `src/decision_viz`
- Viewer config: `src/decision_viz/config/default.yaml`
- Default map: `tools/maps/basemaps/buff_map_field.png`
- Rule-aware structure overlay source: `src/decision_viz/config/default.yaml` -> `structures`
- Scripted route overlay source: `src/decision_viz/config/default.yaml` -> `scripted_path`

## Trace Recording

Tracing is opt-in and stays off for normal competition runs.
`decision_trace_enabled` defaults to `false`; `decision_trace_file` is only used after that switch is explicitly enabled.

```bash
./scripts/start.sh nogate --mode league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

Direct launch:

```bash
ros2 launch behavior_tree sentry_all.launch.py mode:=league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

Leaving `decision_trace_enabled:=false` means no trace file is opened or written, even if a path is accidentally supplied.

Optional indexed wrapper (reads `src/behavior_tree/Scripts/ConfigJson`):

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --list-configs
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --mode league --bt-config chase_only_competition.json --entry nogate
```

Offline decision test (not replay, behavior_tree + mock topics only):

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --offline-decision --mode regional --bt-config regional_competition.json
```

Offline decision + pygame live view:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --offline-decision --mode regional --live-view
```

Before launching live view, `decision_viz.start` auto-cleans stale old `decision_viz.main` viewer processes.

For regional super confrontation timing, use a 7-minute match clock (`420` seconds):

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --offline-decision --mode regional --live-view --match-duration-sec 420
```

One-command fixed regional wrapper:

```bash
python3 scripts/python/start.py
```

Offline mode keeps `/ly/game/is_start` gate enabled by default; press `Start` in viewer/web to publish game-start and enter match phase.
Offline mode enables `runtime_rearm_start_gate:=true` by default. After `Reset`, behavior_tree re-enters start gate,
holds safe-control, publishes Home navigation goal, and waits for next `Start`.
Offline mode also forces `NaviSetting.UseTfGoalBridge=false` via a generated temp config, so `/ly/navi/goal_pos` remains official map coordinates.
Use `--keep-tf-goal-bridge` only when you need transformed bridge output.

Live view now also serves the same pygame frame to HTTP by default (port from YAML `web_stream.port`, default `9000`):

- `http://127.0.0.1:9000/` for local browser
- `http://<your-ip>:9000/` for LAN browser

Override live-view web port:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --offline-decision --mode regional --live-view --live-web-port 9010
```

With the default 100 Hz BT tick rate:

- `decision_trace_every_n_ticks:=1` records about 100 rows/s.
- `decision_trace_every_n_ticks:=5` records about 20 rows/s.
- `decision_trace_every_n_ticks:=10` records about 10 rows/s.

## Viewer

Run from source:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main log/decision_trace.jsonl
```

Open the sample:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main
```

Select a different map:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main \
  log/decision_trace.jsonl \
  --map tools/maps/basemaps/RMUC2026_V1.2.0_topview_cad_field.png
```

Validate a trace without opening pygame:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main log/decision_trace.jsonl --validate-only
```

Disable HTTP stream:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main log/decision_trace.jsonl --no-web-stream
```

Change stream endpoint:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main \
  log/decision_trace.jsonl \
  --web-host 0.0.0.0 \
  --web-port 9000
```

The viewer includes a structure layer for quick rule-context checks:

- Walls / major barrier blocks
- Energy mechanism (rule section `4.3.2.2`)
- Outpost stations (rule section `4.3.2.3`)

These are currently approximated from map artwork + rule figure alignment and are intended for offline decision visualization, not millimeter-level navigation constraints.

YAML default stream config is under `src/decision_viz/config/default.yaml`:

- `web_stream.enabled`
- `web_stream.host`
- `web_stream.port`
- `web_stream.fps`
- `web_stream.jpeg_quality`

Offline match-control defaults are in the same YAML:

- `match_control.duration_sec`
- `match_control.rewind_step_sec`
- `match_control.forward_step_sec`
- `match_control.control_file`

Scripted route defaults are in the same YAML:

- `scripted_path.enabled`
- `scripted_path.goal_ids` / `scripted_path.points_cm`
- `scripted_path.speed_cmps`
- `scripted_path.side` / `scripted_path.loop`
- `scripted_path.show_future`

Live follow mode panel includes `Start`, `Pause`, `+10s`, `-10s`, `Reset` controls for offline mock match timing.
The HTTP page (`/`) provides the same control buttons and writes commands to `match_control.control_file`.
Match clock is rendered as a real-time countdown; pressing `Start` starts countdown immediately.
Live view opens the full viewer immediately, then updates when real trace rows arrive.
When scripted path is enabled, route marker movement uses `speed_cmps` and elapsed match time.
By default, only the visited route plus the current target segment is drawn; set `show_future: true` to draw the full planned route.

After build:

```bash
colcon build --packages-select decision_viz
source install/setup.bash
ros2 run decision_viz decision-viz log/decision_trace.jsonl
```

## Trace Schema

Each line is one JSON object. Important top-level fields:

- `schema`: currently `ly_decision_trace_v1`
- `schema_version`: `2` adds `decision_output`; the viewer still reads older rows without that field
- `event`: `game_start`, `tick`, or `stop`
- `t`: seconds from `gameStartTime`
- `field_cm`: map frame and field size
- `competition_profile`, `strategy_mode`, `team`
- `aim_mode`, `target_armor`, `target_state`
- `decision_output`: stable viewer-facing output model; includes final output kind, topic, goal ID, `goal_pos_cm`, publish flags, and bridge hints
- `navi_goal`: ID, base ID, side, speed, publish flags, and `position_cm`
- `posture`: command, state, runtime desired/current/pending, reason
- `referee`: HP, ammo, time, outpost/base HP, RFID/buff state
- `units`: friend/enemy unit records with type, HP, distance, and `position_cm`
- `runtime_guard`: current fault and recovery state

The viewer uses coordinates in this order:

1. `decision_output.goal_pos_cm`, then legacy `navi_goal.position_cm`, and `units.*.position_cm` from trace.
2. `src/decision_viz/config/default.yaml` goal coordinates.
3. Optional `--points-json` map plugin coordinates when non-zero.

Terrain height is configured, not inferred. `config/default.yaml` contains an approximate 2D elevation overlay based on the visible map artwork. Treat it as a debug layer only until verified against official CAD/rule metadata.

## Maintenance Rules

When decision data changes, update the visualization in the same change:

- New/renamed trace fields: update `src/behavior_tree/src/DecisionTrace.cpp`, `src/decision_viz/decision_viz/model.py`, and `src/decision_viz/decision_viz/trace.py`.
- New output checks: update `src/decision_viz/decision_viz/validation.py`.
- New point IDs or map assumptions: update `src/decision_viz/config/default.yaml`.
- New visual layer or style: update `src/decision_viz/decision_viz/viewer.py` and this document.
- New decision interface docs: add a dated note under `docs/sentry/` or update this dated document with the new date.

Do not move this tool back under `tools/`; it is a maintained source package because it tracks the decision interface.
