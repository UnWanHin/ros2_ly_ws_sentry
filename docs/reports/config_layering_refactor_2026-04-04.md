# Config Layering Refactor (2026-04-04)

## 1. Goal

Refactor startup/config architecture from a single shared `config_file` into layered injection:

- `base` (hardware/camera/solver/io)
- `module` (detector / predictor / outpost / buff)
- `override` (optional final global override)

Keep backward compatibility:

- `config_file:=<yaml>` still works as the final override layer.

## 2. New Config Layout

Added:

- `scripts/config/base_config.yaml`
- `scripts/config/override_config.yaml`
- `src/detector/config/detector_config.yaml`
- `src/predictor/config/predictor_config.yaml`
- `src/outpost_hitter/config/outpost_config.yaml`
- `src/buff_hitter/config/buff_config.yaml`

Removed:

- `scripts/config/stack/*`
- `scripts/config/auto_aim_config_competition.yaml`
- `scripts/config/auto_aim_config_competition.yamlx`

## 3. Launch Injection Priority

All major launch entries now use:

1. base/module files
2. `config_file` (override, applied last)
3. runtime inline launch overrides (offline force flags, etc.)

Main chain (`sentry_all`) per-node injection:

- `gimbal_driver`: `base + override`
- `detector`: `base + detector + override`
- `tracker_solver`: `base + predictor + override`
- `predictor`: `base + predictor + override`
- `outpost_hitter`: `base + outpost + override`
- `buff_hitter`: `base + buff + override`

## 4. Startup Script Changes

Updated scripts now default-inject layered configs from:

- `scripts/config/base_config.yaml`
- `scripts/config/override_config.yaml`
- module configs under `src/*/config/*_config.yaml`

- `scripts/launch/start_sentry_all.sh`
- `scripts/launch/start_sentry_showcase.sh`
- `scripts/launch/start_sentry_chase_only.sh`
- `scripts/launch/start_sentry_navi_debug.sh`
- `scripts/launch/armor_test.sh`
- `scripts/launch/start_autoaim_debug.sh`
- `scripts/launch/shooting_table_calib.sh`

## 5. Reader/Runtime Config Code Changes

### 5.1 outpost_hitter

Added YAML-driven fire parameters (with defaults):

- `outpost_config.fire.bullet_speed`
- `outpost_config.fire.pitch_bias_deg`
- `outpost_config.fire.max_yaw_delta_deg`
- `outpost_config.fire.time_delay_sec`

Applied in runtime:

- bullet speed no longer hardcoded at call-site
- pitch bias and yaw delta thresholds now configurable
- muzzle solver time delay configurable via new setter

### 5.2 buff_hitter

Added layered YAML override support under `buff_config.*` while keeping JSON base:

- dynamic bullet-speed filter parameters
- enemy color / mode switch / two-target policy
- red/blue model paths
- auto-mode thresholds
- static and periodic shoot-table calibration coefficients

Also updated buff ROS node options to support parameter overrides robustly.

### 5.3 gimbal_driver

IO parameter read path now supports both slash and dot naming:

- `io_config/use_virtual_device` + `io_config.use_virtual_device`
- `io_config/device_name` + `io_config.device_name`
- `io_config/baud_rate` + `io_config.baud_rate`
- posture repeat keys likewise

## 6. Package Install Changes

`behavior_tree/CMakeLists.txt` now installs full `scripts/config/` directory into:

- `share/behavior_tree/config/`

So launch defaults can resolve layered config files from installed package paths.

## 7. Selfcheck Sync

Updated `scripts/selfcheck/sentry.sh`:

- static checks include base/override + module config files
- launch mode hints now inspect split base/detector config files
- camera SN consistency check switched to base config

## 8. Notes

- Existing user workflows that pass `config_file:=...` remain available.
- Recommended new workflow is to edit module-specific files under each package `src/<module>/config/`.
