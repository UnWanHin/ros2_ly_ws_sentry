# decision_viz

Offline pygame viewer for sentry behavior-tree decision traces.

Updated: 2026-04-27

## Scope

- Runs offline from JSONL traces produced by `behavior_tree`.
- Normalizes trace rows into `DecisionOutput` records so future decision internals can change while the viewer stays centered on the final goal output.
- Draws a 2D map, navigation goals, recent goal path, friendly/enemy units, HP bars, strategy, aim mode, posture, target, ammo, terrain overlays, and recent decision changes.
- Draws rule-aware structure overlays (walls, energy mechanism, outposts) on top of the 2D map.
- Adds offline match-time control for super confrontation regional tests: start, pause, rewind, forward, reset.
- Uses `tools/maps/basemaps/buff_map_field.png` by default, but the map is selectable.
- Keeps window size, field size, colors, point coordinates, terrain overlays, unit styles, layer switches, and web stream defaults in `config/default.yaml`.

## Record

Tracing is disabled unless `decision_trace_enabled:=true`.

```bash
./scripts/start.sh nogate --mode league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

At 100 Hz BT tick rate, `decision_trace_every_n_ticks:=5` records about 20 rows per second.

### Which Decision Config Is Running

`./scripts/start.sh ... --mode league` resolves to:

- `bt_config_file := Scripts/ConfigJson/league_competition.json`

`--mode regional` resolves to `regional_competition.json`, and `--mode showcase` resolves to `showcase_competition.json`.

If you pass `bt_config_file:=...`, that explicit file overrides the mode default.

### Indexed Recorder Start Script

Use the wrapper to index `src/behavior_tree/Scripts/ConfigJson` and start trace recording:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --list-configs
```

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start \
  --mode league \
  --bt-config chase_only_competition.json \
  --entry nogate
```

In normal mode (without `--offline-decision`), the wrapper enables trace and prints the exact forwarded command.
It writes trace to `log/decision_trace_<timestamp>.jsonl` by default.

Auto-open pygame after record exits:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --mode league --play
```

Open pygame while decision is running:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --mode league --live-view
```

When live view starts, it also opens HTTP frame streaming by default (port from YAML `web_stream.port`, default `9000`):

- `http://127.0.0.1:9000/` (local browser)
- `http://<your-ip>:9000/` (LAN browser)

Override live-view stream port from wrapper:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start --mode league --live-view --live-web-port 9010
```

### Offline Decision Test (Not Replay)

Run behavior-tree decision offline with built-in mock topic publishers (no detector/gimbal/predictor/outpost/buff nodes):

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start \
  --offline-decision \
  --mode league \
  --bt-config chase_only_competition.json
```

Add pygame live view in offline mode:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start \
  --offline-decision \
  --mode league \
  --live-view
```

For super confrontation regional logic, keep `--mode regional` and use a 7-minute match clock (`420` seconds):

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --match-duration-sec 420
```

Offline mode now keeps `/ly/game/is_start` gate enabled by default (no `debug_bypass_is_start`).
Use the viewer/web `Start` control to publish game-start and begin countdown.

By default, offline mode also writes a temporary BT config with `NaviSetting.UseTfGoalBridge=false`,
so goal position publish uses official map coordinates (no transformed bridge output).
Use `--keep-tf-goal-bridge` if you explicitly need transformed output.

By default, `--offline-decision` keeps trace off. Add `--trace-on` if you want JSONL output at the same time:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start \
  --offline-decision \
  --mode league \
  --trace-on
```

You can choose which target stream is active in offline test:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.start \
  --offline-decision \
  --mock-target predictor
```

## Playback From Source Tree

```bash
python3 -m decision_viz.main log/decision_trace.jsonl
```

If `PYTHONPATH` does not include this package:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main log/decision_trace.jsonl
```

Open the bundled sample:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main
```

Use a different map or config:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main \
  log/decision_trace.jsonl \
  --map tools/maps/basemaps/RMUC2026_V1.2.0_topview_cad_field.png \
  --config src/decision_viz/config/default.yaml
```

Disable web streaming:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main log/decision_trace.jsonl --no-web-stream
```

Change stream host/port:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main \
  log/decision_trace.jsonl \
  --web-host 0.0.0.0 \
  --web-port 9000
```

Stream defaults can be configured in YAML:

- `src/decision_viz/config/default.yaml` -> `web_stream.enabled`
- `src/decision_viz/config/default.yaml` -> `web_stream.host`
- `src/decision_viz/config/default.yaml` -> `web_stream.port`
- `src/decision_viz/config/default.yaml` -> `web_stream.fps`
- `src/decision_viz/config/default.yaml` -> `web_stream.jpeg_quality`
- `src/decision_viz/config/default.yaml` -> `structures` (walls/energy mechanism/outposts)
- `src/decision_viz/config/default.yaml` -> `match_control` (duration/control-file/button step)

Headless smoke test:

```bash
SDL_VIDEODRIVER=dummy PYTHONPATH=src/decision_viz python3 -m decision_viz.main --smoke-test
```

Offline consistency validation:

```bash
PYTHONPATH=src/decision_viz python3 -m decision_viz.main log/decision_trace.jsonl --validate-only
```

After building this package:

```bash
colcon build --packages-select decision_viz
source install/setup.bash
ros2 run decision_viz decision-viz log/decision_trace.jsonl
```

## Dependencies

```bash
python3 -m pip install -r src/decision_viz/requirements.txt
```

`PyYAML` is already present in the current workspace environment. `pygame` may need to be installed locally.

## Controls

- `Space`: play or pause
- `Left` / `Right`: step one record
- `PageUp` / `PageDown`: jump by `timeline.jump_step`
- `Home` / `End`: jump to start or end
- `+` / `-`: playback speed
- `L`: toggle point/unit labels
- Mouse click the timeline to seek
- In follow mode with offline mock control: click panel buttons for `Start`, `Pause`, `+10s`, `-10s`, `Reset`
- The HTTP page (`/`) now has the same control buttons and writes commands to `match_control.control_file`
- Keyboard shortcuts for offline mock control: `S` start, `P` pause, `[` rewind, `]` forward, `R` reset
- Match clock is a real-time countdown: `Start` begins countdown immediately, then syncs with incoming trace `time_left`
- In follow waiting screen (before first trace), press `S` or `Enter` to send start gate and unblock trace generation

## Maintenance Contract

When a behavior-tree decision output changes, update this package in the same PR:

- `src/behavior_tree/src/DecisionTrace.cpp`
- `src/decision_viz/decision_viz/model.py`
- `src/decision_viz/decision_viz/trace.py`
- `src/decision_viz/decision_viz/validation.py` if output checks need to change
- `src/decision_viz/config/default.yaml` if field, map, point, unit, layer, or style assumptions changed
- `docs/sentry/decision_visualization_2026-04-27.md` or a newer dated document

Do not rely on `tools/maps/map_plugin.json` for required point positions until it has non-zero calibrated coordinates. The viewer currently uses trace positions first, YAML points second, and optional map plugin coordinates third.
Terrain height is an approximate YAML overlay, not inferred from the image at runtime. Use official CAD/rules or measured map metadata before using it for path-cost decisions.
