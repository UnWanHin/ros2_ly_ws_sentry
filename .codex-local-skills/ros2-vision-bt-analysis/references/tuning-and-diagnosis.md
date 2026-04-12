# Tuning And Diagnosis

Use this checklist only after chain mapping is complete.

## Fast Isolation Sequence

1. Verify predictor output freshness and rate.
2. Verify behavior_tree consumes predictor and publishes `/ly/control/angles`.
3. Verify gimbal_driver receives `/ly/control/angles` unchanged.
4. If lag remains, tune predictor/tracker first, then BT no-target behavior.

## Predictor Parameters (Most Relevant)

- `motion_model.q_omega_accel_base`
  - Higher value: faster yaw dynamics response, less lag, potentially noisier.
- `motion_model.r_scale`
  - Lower value: trust observation more, less smoothing, potentially jittery.
- `motion_model.boundary_fit_alpha`
  - Higher value: adaptive R reacts faster to residual change.
- `motion_model.boundary_scale_min` / `motion_model.boundary_scale_max`
  - Tight at `1.0/1.0`: disable adaptive R scaling effect.
  - Wide range: stronger adaptive smoothing behavior.

## Runtime A/B Override Pattern

Use launch overrides before YAML edits:

```bash
./scripts/launch/armor_only_test.sh -- \
  motion_model.boundary_scale_min:=1.0 \
  motion_model.boundary_scale_max:=1.0 \
  motion_model.r_scale:=0.02 \
  motion_model.q_omega_accel_base:=1.0
```

If still laggy, move `q_omega_accel_base` upward in small steps (e.g., `1.0 -> 2.0 -> 5.0`).

## BT-Side Checks (No-Target Behavior)

- `AimDebug.StopScan`
- `AimDebug.ReuseLatchedAnglesOnNoTarget`
- chase lost-target hold behavior

Symptoms suggesting BT-side issue:
- predictor updates are fresh, but `/ly/control/angles` sticks to old angle.
- behavior differs only by BT json profile/mode.

## Tracker-Side Checks

- `tracker_config.use_matcher_tracking`
- `tracker_config.use_whole_car_matcher`
- matcher internal smoothing constants (`alpha`) in matcher headers

If disabling matcher has little impact, prioritize predictor + BT investigation.

## Evidence Requirements

Always provide at least one of:
- exact file/line references proving logic branch.
- runtime signal proving effective params (e.g., `ros2 param get`).
- topic chain proof (`ros2 topic info -v`, node list, and observed publisher/subscriber).
