---
name: ros2-sentry-bt-framework
description: Read and modify this ROS2 sentry decision stack safely across launch, BT XML, Application/GameLoop, config JSON, and topic contracts (including chase and TF-goal bridge modes).
---

# ROS2 Sentry BT Framework

## Overview

Use this skill when the task involves `behavior_tree` decision behavior, mode/profile selection, navigation goal output mode, chase logic, or TF goal bridge integration.

This project is a BT-driven decision stack:
- Launch selects profile/config
- `behavior_tree` loads JSON + `Scripts/main.xml`
- BT nodes call `Application` methods
- `PublishMessage` emits final control/nav topics

Read order (always):
1. Entry launch/script and effective runtime args
2. Effective BT config JSON
3. BT XML structure
4. `Application.cpp` init path and `GameLoop.cpp` runtime path
5. `SubscribeMessage.cpp` and `PublishMessage.cpp` topic contracts

## Key Files

- Entry and launch:
  - `scripts/launch/start_sentry_*.sh`
  - `src/behavior_tree/launch/sentry_all.launch.py`
  - `src/navi_tf_bridge/launch/decision_chase.launch.py`
- BT and app core:
  - `src/behavior_tree/Scripts/main.xml`
  - `src/behavior_tree/include/BTNodes.hpp`
  - `src/behavior_tree/src/Application.cpp`
  - `src/behavior_tree/src/BehaviorTree.cpp`
  - `src/behavior_tree/src/GameLoop.cpp`
  - `src/behavior_tree/src/Configuration.cpp`
  - `src/behavior_tree/src/WaitBeforeGame.cpp`
  - `src/behavior_tree/src/SubscribeMessage.cpp`
  - `src/behavior_tree/src/PublishMessage.cpp`
- Decision config:
  - `src/behavior_tree/Scripts/ConfigJson/*.json`
- TF goal bridge:
  - `src/navi_tf_bridge/config/tf_config.yaml`
  - `src/navi_tf_bridge/src/target_rel_to_goal_pos_node.cpp`

Load details from:
- `references/decision-framework.md`

## Workflow

1. Resolve effective mode/profile/config first.
2. Confirm whether the system is in goal-ID mode or XY goal mode.
3. Confirm whether TF goal bridge is enabled.
4. Trace where final `/ly/navi/goal_pos` is produced.
5. Keep topic/schema stable; use minimal local diffs.
6. Verify with targeted build and launch arg inspection.

## Output Mode Matrix

Use this matrix before any change:

- `UseXY=false`
  - BT publishes `/ly/navi/goal` (ID mode)
- `UseXY=true` and `UseTfGoalBridge=false`
  - BT publishes `/ly/navi/goal_pos` directly
- `UseXY=true` and `UseTfGoalBridge=true`
  - Relative chase: `/ly/navi/target_rel -> navi_tf_bridge -> /ly/navi/goal_pos`
  - Raw map points: `/ly/navi/goal_pos_raw -> navi_tf_bridge -> /ly/navi/goal_pos`

## Guardrails

- Do not rewire launch/data chain broadly.
- Do not change topic names unless explicitly required.
- Do not hardcode mode-specific values into `.sh` when JSON/YAML can own them.
- Keep `behavior_tree` as a single decision/control source unless task explicitly asks otherwise.

## Validation

Minimum checks:
- `colcon build --packages-select behavior_tree navi_tf_bridge`
- `python3 -m py_compile` for changed launch/python files
- `ros2 launch ... --show-args` to confirm parameter injection

