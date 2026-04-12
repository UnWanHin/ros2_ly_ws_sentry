---
name: ros2-vision-bt-analysis
description: Diagnose ROS2 sentry auto-aim issues across visual solving, tracker, predictor, behavior_tree, and gimbal links with evidence-first workflow. Use when users report laggy gimbal follow, over-smoothing, wrong no-target behavior, target switch delay, or suspect BT/topic-chain faults.
---

# Ros2 Vision Bt Analysis

## Overview

Use this skill to identify root cause in `detector -> tracker_solver -> predictor -> behavior_tree -> gimbal_driver` without broad refactor or blind parameter sweeping.  
Always prefer launch/config/code evidence first, then runtime topic/param verification, then minimal-change recommendations.

## Workflow

1. Map the active chain from startup entry to final control publisher.
2. Confirm effective config and launch arguments (especially debug toggles and BT json profile).
3. Locate all smoothing/holding points in tracker, predictor, and BT.
4. Attribute primary cause with evidence (file + line, or runtime signal).
5. Recommend the smallest safe change order: runtime override -> YAML tuning -> code change.

## Chain Mapping

Resolve actual startup path first:
- `scripts/start.sh` / `scripts/launch/*.sh`
- `src/behavior_tree/launch/*.launch.py`
- Node-level parameter injection in launch files

When checking mode differences, explicitly compare:
- `debug_bypass_is_start`
- `mode`, `competition_profile`, `bt_config_file`
- `use_behavior_tree`, `offline`
- test wrappers (e.g., armor-only scripts) vs competition entry

## Root-Cause Attribution Rules

Treat as predictor/tracker issue first if:
- target angle updates exist but are slow/smooth under fast lateral target motion
- behavior_tree consumes `/ly/predictor/target` correctly
- no-target branch is not dominating runtime

Treat as BT issue first if:
- behavior on no-target is unexpected (latched-angle reuse, hold-current, or scan branch)
- `/ly/control/angles` diverges from predictor output even when predictor is fresh
- mode-specific json changes behavior (`StopScan`, `ReuseLatchedAnglesOnNoTarget`, chase/lost-target settings)

Treat as topic-chain issue first if:
- publisher exists but subscriber missing/QoS incompatible
- message timestamps stale or frame mismatch
- launch toggles disable critical nodes

## High-Impact Smoothing Hotspots

Prioritize these checkpoints:
- Tracker matcher smoothing and whole-car matcher logic
- Predictor process/measurement noise and adaptive boundary scaling
- Predictor publish gating (`publish_only_on_new_tracker_frame`, freshness requirement)
- BT no-target branch (`StopScan`, latched reuse, hold-current logic)
- Gimbal command path integrity (`/ly/control/angles` producer and final writer)

Load this reference when tuning:
- `references/tuning-and-diagnosis.md`

## Output Contract

Respond in this order:
1. Evidence: concrete file/line or runtime proof.
2. Conclusion: primary cause + secondary factors.
3. Minimal next action: one test command override or one YAML delta set.
4. Risk note: what remains unverified.

Never claim "BT issue" or "predictor issue" without a direct chain of evidence.
