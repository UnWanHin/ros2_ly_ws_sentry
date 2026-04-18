# Decision Framework Reference

## 1) Runtime Chain

Primary project chain:

`detector -> tracker_solver -> predictor -> behavior_tree -> gimbal_driver`

Navigation-related chain in TF mode:

`behavior_tree(/ly/navi/target_rel or /ly/navi/goal_pos_raw) -> navi_tf_bridge -> /ly/navi/goal_pos`

## 2) behavior_tree Runtime Structure

Entry:
- `src/behavior_tree/main.cpp`

Initialization:
- `src/behavior_tree/src/Application.cpp`
  - Resolve `bt_tree_file` and `bt_config_file`
  - Init subscriptions and publishers
  - Load config (`ConfigurationInit`)
  - Register BT nodes and load XML tree

Main loop:
- `src/behavior_tree/src/WaitBeforeGame.cpp`
  - gate on `/ly/game/is_start` unless bypass/timeout
- `src/behavior_tree/src/GameLoop.cpp`
  - `spin_some`
  - `TreeTickGuarded`
  - `PublishTogether` at BT tail

## 3) BT XML Dispatch

`src/behavior_tree/Scripts/main.xml` main sequence:

1. `UpdateGlobalData`
2. `SelectAimMode`
3. `SelectStrategyMode`
4. `StrategyDispatch` subtree
5. `PreprocessData`
6. `SelectAimTarget`
7. `SelectPosture`
8. `PublishAll`

`StrategyDispatch` fallback includes:
- Recovery check
- `HitSentry`, `HitHero`, `LeagueSimple`, `Protected`, `NaviTest`

## 4) Config Ownership

Decision JSONs:
- `src/behavior_tree/Scripts/ConfigJson/*.json`

`sentry_all.launch.py` resolves:
- mode
- competition profile
- BT config path

`Configuration.cpp` deserializes and sanitizes:
- `NaviSetting`
- `Chase`
- `LeagueStrategy`
- `Posture`
- `AimTargetPriority`

## 5) Navigation Output Modes

### ID mode
- condition: `UseXY=false`
- output: `/ly/navi/goal` (uint8 ID) + `/ly/navi/speed_level`

### XY direct mode
- condition: `UseXY=true`, `UseTfGoalBridge=false`
- output: BT direct `/ly/navi/goal_pos`

### XY TF bridge mode
- condition: `UseXY=true`, `UseTfGoalBridge=true`
- BT output becomes bridge input:
  - relative chase: `/ly/navi/target_rel`
  - raw map points: `/ly/navi/goal_pos_raw`
- bridge final output:
  - `/ly/navi/goal_pos`

Related files:
- `src/behavior_tree/src/PublishMessage.cpp`
- `src/behavior_tree/src/GameLoop.cpp`
- `src/navi_tf_bridge/src/target_rel_to_goal_pos_node.cpp`

## 6) Topic Contract Hotspots

Inbound to BT:
- `/ly/predictor/target`
- `/ly/outpost/target`
- `/ly/buff/target`
- `/ly/gimbal/angles`
- `/ly/game/*`
- `/ly/position/data`

Outbound from BT:
- `/ly/control/angles`
- `/ly/control/firecode`
- `/ly/control/vel`
- `/ly/control/posture`
- `/ly/navi/goal` or `/ly/navi/goal_pos` or `/ly/navi/goal_pos_raw`
- `/ly/navi/target_rel` (when chase relative-topic mode is enabled)

## 7) Known Behavior Details

- `SubscribeMessage.cpp` currently uses coordinate conversion for position data:
  - `Y = 1500 - y`
- `WaitBeforeGame` publishes safe zero-control while waiting start gate.
- In chase relative-topic mode, BT should avoid direct `/ly/navi/goal_pos` contention.

## 8) Practical Checks

1. Build:
- `colcon build --packages-select behavior_tree navi_tf_bridge`

2. Launch arg inspection:
- `ros2 launch behavior_tree sentry_all.launch.py --show-args`
- `ros2 launch navi_tf_bridge decision_chase.launch.py --show-args`

3. Topic ownership:
- Verify single producer for `/ly/control/*`
- Verify expected producer for `/ly/navi/goal_pos`

