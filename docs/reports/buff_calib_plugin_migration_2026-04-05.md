# Buff Shooting Table Calibration Plugin Migration (2026-04-05)

## Scope

This change adds a ROS2-style buff calibration plugin flow for sentry, while keeping runtime control interfaces unchanged.

## Interface Impact

No runtime control interface was changed.

Unchanged control paths:
- `/ly/buff/target`
- `/ly/control/angles`
- `/ly/control/firecode`

Added debug-only path:
- New message: `auto_aim_common/msg/BuffDebug.msg`
- New topic from `buff_hitter`: `/ly/buff/debug`

`/ly/buff/debug` is only for calibration/diagnostics and does not drive fire-control outputs.

## Implemented Changes

### 1) Buff debug data export from buff_hitter

Updated file:
- `src/buff_hitter/main.cpp`

Added:
- Publisher `LY_DEF_ROS_TOPIC(ly_buff_debug, "/ly/buff/debug", auto_aim_common::msg::BuffDebug)`
- `PubDebug(...)` helper
- Publish points in success/failure branches of buff solve loop

Exported fields include:
- solver setpoint yaw/pitch
- mode (small/big)
- rotation angle
- distance/height
- predicted target xyz (meters)

### 2) New ROS2 calibration package

Package:
- `src/buff_shooting_table_calib`

Type:
- `rclpy` node (plugin style, no control takeover)

Node:
- `script/buff_shooting_table_calib_node.py`

Behavior:
- subscribes `/ly/buff/debug`
- subscribes `/ly/gimbal/angles`
- subscribes `/ly/gimbal/firecode`
- samples one record on fire rising edge by default
- writes CSV under `~/workspace/record` by default

CSV schema:
- `timestamp,mode,status,firecode,target_yaw,target_pitch,gimbal_yaw,gimbal_pitch,relative_yaw,relative_pitch,rotation_angle,distance_m,height_m,target_x_m,target_y_m,target_z_m`

Config:
- `src/buff_shooting_table_calib/config/buff_shooting_table_calib_config.yaml`

Launch:
- `src/buff_shooting_table_calib/launch/buff_shooting_table_calib.launch.py`

### 3) Offline fitting tools

Added tools:
- `scripts/tools/fit_buff_static_shoot_table.py`
- `scripts/tools/fit_big_buff_moving_compensation.py`

Output target keys (override YAML):
- static:
  - `buff_config.static_shoot_table_adjust_enable`
  - `buff_config.static_pitch_adjust_coeffs`
  - `buff_config.static_yaw_adjust_coeffs`
- periodic:
  - `buff_config.periodic_shoot_table_adjust_enable`
  - `buff_config.periodic_apply_on_big_buff_only`
  - `buff_config.periodic_pitch_adjust_coeffs`
  - `buff_config.periodic_yaw_adjust_coeffs`

Write-back policy:
- Manual by default.
- Optional `--write-config` exists and creates timestamp backup.

### 4) Scripts integration

Added:
- launch entry: `scripts/launch/buff_shooting_table_calib.sh`
- debug entry: `scripts/debug/buff_shooting_table_calib.sh`
- menu entry in `scripts/debug.sh`

Docs/script index updates:
- `scripts/README.md`
- `scripts/FUNCTION_GUIDE.md`
- `scripts/selfcheck/pc.sh` package list includes `buff_shooting_table_calib`

## Typical Usage

Collect calibration samples:

```bash
./scripts/debug.sh buff-shooting-table-calib --calib-mode periodic --csv-strategy latest
```

Fit static compensation from latest CSV:

```bash
./scripts/launch/buff_shooting_table_calib.sh --fit-static-latest
```

Fit periodic compensation from latest CSV:

```bash
./scripts/launch/buff_shooting_table_calib.sh --fit-periodic-latest
```

## Notes

- This migration does not include automatic online write-back to runtime configs.
- Fitted parameters should be reviewed and then merged manually into target YAML.
