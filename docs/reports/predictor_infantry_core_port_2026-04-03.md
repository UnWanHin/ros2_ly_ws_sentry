# Predictor Core Port Report (2026-04-03)

## Scope
This change ports infantry-side core algorithm ideas into the ROS2 sentry workspace while keeping external interfaces stable.

Applied items:
- P0: 10D observation chain (`N_Y=10`) in predictor core.
- P0: Boundary-residual adaptive measurement covariance `R` (yaw/pitch boundary blocks).
- P0: Keep ROS2 numerical safety framework (`dt` clamp in `time_ekf`, Joseph covariance update in `ekf`).
- P0: Parameter landing in runtime YAML used by competition launcher.
- P1: Anti-rotate armor switching in controller (configurable).
- P1: Configurable armor visible-angle threshold (replacing hard-coded `pi/3`).
- P1: 8081 overlay support for predictor center point from `/ly/predictor/debug` (configurable switch).
- P1: Full 4-armor visualization topic/message for 8081 overlay (`/ly/predictor/vis`).
- Follow-up: buff_hitter bullet-speed chain switched from fixed value to dynamic uplink + EMA smoothing.
- Follow-up: buff_hitter core modules (`BuffCalculator/BuffDetector/BuffController`) aligned to infantry implementation shape.

## Files Changed
- `src/auto_aim_common/msg/PredictorArmorVis.msg`
- `src/auto_aim_common/msg/PredictorCarVis.msg`
- `src/auto_aim_common/msg/PredictorVis.msg`
- `src/auto_aim_common/CMakeLists.txt`
- `src/predictor/include/predictor/motion_model.hpp`
- `src/predictor/src/motion_model.cpp`
- `src/predictor/src/predictor.cpp`
- `src/predictor/include/controller/controller.hpp`
- `src/predictor/src/controller.cpp`
- `src/predictor/predictor_node.cpp`
- `src/detector/detector_node.cpp`
- `src/gimbal_driver/main.cpp`
- `src/buff_hitter/main.cpp`
- `src/buff_hitter/config/config.json`
- `src/buff_hitter/module/BuffCalculator.hpp`
- `src/buff_hitter/src/BuffCalculator.cpp`
- `src/buff_hitter/module/BuffDetector.hpp`
- `src/buff_hitter/src/BuffDetector.cpp`
- `src/buff_hitter/src/BuffController.cpp`
- `scripts/config/base_config.yaml`
- `scripts/config/override_config.yaml`
- `src/detector/config/detector_config.yaml`
- `src/predictor/config/predictor_config.yaml`

## Detailed Changes

### 1) 10D observation chain (`N_Y=10`)
- Expanded observation dimension from 7 to 10.
- Added pitch boundary observations:
  - `measure[7] = pitch_top`
  - `measure[8] = pitch_bottom`
  - `measure[9] = pitch_center`
- `measureFunc` now predicts yaw boundary and pitch boundary observations from state.
- `predictor::update` now populates top/bottom/center pitch from car bounding box plus gimbal state.
- `world2model` now maps all 10 observation channels into model observation space.

### 2) Adaptive `R` for boundary residuals
- Added per-update residual-based adaptive scaling for boundary measurement covariance:
  - yaw boundary block (`[4..6]`) uses `yaw_boundary_var_ema`.
  - pitch boundary block (`[7..9]`) uses `pitch_boundary_var_ema`.
- Scaling uses EMA + clamp:
  - `boundary_fit_alpha`
  - `boundary_scale_min`
  - `boundary_scale_max`
- Each update starts from base `R`, then adapts boundary sub-blocks before EKF update.

### 3) Numerical safety kept
- `time_ekf` `dt` clamp behavior retained.
- `ekf` Joseph-form covariance update retained.
- No rollback to unstable covariance update.

### 4) Controller anti-rotate (configurable)
- Added anti-rotate armor switching in controller:
  - Trigger by `|omega| > omega_threshold`.
  - Compute `jump_angle` from distance/radius/omega/response_speed.
  - Switch armor id by rotation direction when crossing jump angle.
- Parameters are loaded from YAML under `controller_config.anti_rotate.*`.

### 5) Visible-angle threshold configurable
- Replaced hard-coded `pi/3` in predictor armor availability decision with runtime parameter:
  - `predictor.visible_half_angle_rad`

### 6) 8081 predictor-center overlay (configurable)
- Detector subscribes to `/ly/predictor/debug`.
- Added optional center-point overlay drawing on detector stream image (the image that feeds 8081 stream).
- Overlay projection uses current gimbal angles and loaded solver extrinsics/intrinsics.
- No external topic/message interface changes.

### 7) Full 4-armor visualization topic/message and overlay
- Added new messages in `auto_aim_common`:
  - `PredictorArmorVis.msg`
  - `PredictorCarVis.msg`
  - `PredictorVis.msg`
- Predictor now publishes `/ly/predictor/vis` with:
  - per-car center
  - per-armor center/id/status (`NONEXIST/UNSEEN/AVAILABLE`)
- Detector subscribes to `/ly/predictor/vis` and draws:
  - car center point
  - four armor points with status color overlay on 8081 stream
- This path is controlled by YAML switch `detector_config.overlay_predictor_full`.

### 8) Bullet-speed chain fix (remove hard-coded return and re-enable smoothing)
- `gimbal_driver` now publishes measured bullet speed directly:
  - `msg.data = data.BulletSpeed / 100.0f`
  - removed hard-coded overwrite `23.0f`
- `predictor/controller` no longer forces fly-time speed to fixed `23.0`.
- Controller now uses runtime bullet speed with infantry-style EMA:
  - valid when incoming speed is above `min_bullet_speed`
  - update: `v = alpha * v_in + (1 - alpha) * v_prev`
  - fly-time uses `max(v, min_bullet_speed)` for safety

### 9) Buff_hitter bullet-speed chain fix (fixed -> dynamic)
- `buff_hitter` now subscribes `/ly/bullet/speed` and uses that as solver input source.
- Added infantry-style bullet-speed smoothing in `buff_hitter`:
  - validity gate: incoming speed `> min_bullet_speed`
  - update: `v = alpha * v_in + (1 - alpha) * v_prev`
  - safety floor: `max(v, min_bullet_speed)` before `BuffCalculator::calculate(...)`
- Added config keys under `buff` in `src/buff_hitter/config/config.json`:
  - `dynamic_bullet_speed_enable`
  - `default_bullet_speed`
  - `min_bullet_speed`
  - `bullet_speed_alpha`

### 10) Buff_hitter core migration from infantry modules
- Replaced ROS2-side buff core implementation with infantry-side module structure:
  - `BuffCalculator` now uses infantry-style function signature:
    - `calculate(frame, camera_points, buff_mode, bullet_speed, reload_big_buff)`
  - `BuffDetector` now supports dual-model path and color-select inference:
    - constructor `(red_model_path, blue_model_path)`
    - `buffDetect(frame, enemy_color)`
    - multi-candidate handling and camera-point selection API (`getCameraPointsByIndex`)
  - `BuffController` implementation restored (yaw nearest-angle normalization and validity fallback)
- Main node integration:
  - reads infantry-compatible buff config items (`default_mode`, `reload_big_buff`, dual model paths)
  - supports runtime mode switch via `/ly/ra/mode` (`0=follow default`, `1=small`, `2=big`)
  - `default_mode=0` now means upper-computer auto mode (vision-side mode estimation, no lower-computer `aim_request` dependency)
  - supports dual-target scheduling in buff loop (primary/secondary index, rising-edge switch-to-second, timeout reset)
  - applies `BuffController` output before publish (`valid` gate + nearest-yaw normalization)
  - keeps ROS2 external topic interface unchanged (`/ly/buff/target`)
  - keeps armor predictor/tracker chain untouched
- Compatibility guards added:
  - `FPS` falls back to legacy `buff_calculate_FPS` when absent
  - optional params (`AFTER_PITCH`, `AFTER_YAW`, `force_stable`) default safely

## New/Updated Runtime Parameters

### Predictor / Motion Model
- `motion_model.boundary_fit_alpha`
- `motion_model.boundary_scale_min`
- `motion_model.boundary_scale_max`
- `predictor.visible_half_angle_rad`

### Controller
- `controller_config.anti_rotate.enable`
- `controller_config.anti_rotate.omega_threshold`
- `controller_config.anti_rotate.response_speed`
- `controller_config.anti_rotate.base_jump_angle_rad`
- `controller_config.anti_rotate.min_jump_angle_rad`
- `controller_config.anti_rotate.min_radius`

### Detector (8081 overlay)
- `detector_config.overlay_predictor_center`
- `detector_config.overlay_predictor_full`
- `detector_config.overlay_predictor_timeout_ms`

## Interface Compatibility
- Kept unchanged:
  - `/ly/tracker/results`
  - `/ly/predictor/target`
  - `/ly/predictor/debug`
- Added:
  - `/ly/predictor/vis`
  - detector/tracker/predictor launch-level interfaces
- This patch changes internal algorithm and optional visualization behavior only; control data path is unchanged.

## Build Verification
Command used:
```bash
colcon build --packages-select auto_aim_common predictor detector --event-handlers console_direct+
```
Result:
- `auto_aim_common`: success
- `predictor`: success
- `detector`: success

Additional command used:
```bash
colcon build --packages-select buff_hitter --event-handlers console_direct+
```
Result:
- `buff_hitter`: success

## Usage Notes
- To enable predictor center overlay on 8081 stream:
  - set `detector_config.overlay_predictor_center: true`
- To enable full 4-armor predictor overlay on 8081 stream:
  - set `detector_config.overlay_predictor_full: true`
- To use stricter visible-angle gate similar infantry `pi/6`:
  - set `predictor.visible_half_angle_rad: 0.52359877559`
- To disable anti-rotate fallback quickly:
  - set `controller_config.anti_rotate.enable: false`
