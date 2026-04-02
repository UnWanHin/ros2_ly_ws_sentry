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
- `scripts/config/auto_aim_config_competition.yaml`
- `src/detector/config/auto_aim_config.yaml`

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

## Usage Notes
- To enable predictor center overlay on 8081 stream:
  - set `detector_config.overlay_predictor_center: true`
- To enable full 4-armor predictor overlay on 8081 stream:
  - set `detector_config.overlay_predictor_full: true`
- To use stricter visible-angle gate similar infantry `pi/6`:
  - set `predictor.visible_half_angle_rad: 0.52359877559`
- To disable anti-rotate fallback quickly:
  - set `controller_config.anti_rotate.enable: false`
