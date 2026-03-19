# `ros2_ly_ws` vs `ros2_ly_ws_sentary` 决策系统对比

日期：2026-03-18  
范围：主要比较两个工作区里 `behavior_tree` 决策层，以及与其强相关的启动参数、比赛配置、控制收口行为。  
结论先说：

- `ros2_ly_ws` 的决策仍然偏“手写流程 + 少量 BT 外壳”，核心逻辑主要直接写在 `GameLoop.cpp` 里。
- `ros2_ly_ws_sentary` 已经演进成“BT 主调度 + 多比赛 profile + 姿态状态机 + 运行时保护”的结构。
- 如果后续只维护一套，建议以 `ros2_ly_ws_sentary` 为主，`ros2_ly_ws` 只保留作历史对照。

---

## 1. 决策架构的主要区别

| 对比项 | `ros2_ly_ws` | `ros2_ly_ws_sentary` | 影响 |
| --- | --- | --- | --- |
| 决策主入口 | `GameLoop()` 里手动调用 `SetAimMode -> CheckDebug -> ProcessData -> SetPositionRepeat -> SetAimTarget` | `GameLoop()` 中执行 `TreeTickGuarded()`，由 BT XML 驱动节点顺序 | `sentary` 的调度关系更清晰，扩展点更稳定 |
| BT 是否真正主导 | 旧 `main.xml` 只有 `SetAimTargetFromAim` 和 `SetNaviPosition`，而且旧 `GameLoop.cpp` 中 `BTree.tickRoot()` 还是注释掉的 | 新 `main.xml` 明确是 `UpdateGlobalData -> SelectAimMode -> SelectStrategyMode -> StrategyDispatch -> PreprocessData -> SelectAimTarget -> SelectPosture -> PublishAll` | `sentary` 里行为树不再只是壳，已经变成主执行框架 |
| 策略调度方式 | 主要靠 `SetPositionRepeat()` 里的 if/else 链 | 先由 `SelectStrategyMode` 选模式，再由 `StrategyDispatch` 子树分发到不同策略节点 | `sentary` 更接近“配置驱动的策略框架” |
| 决策配置组织 | 只有 `config.json` + 若干旧 `ConfigJson/*.json` | `config.json` 之外新增 `regional_competition.json`、`league_competition.json`、`showcase_competition.json`、`navi_debug_competition.json` | `sentary` 支持按场景切换，不再靠手改同一份配置 |
| 开赛门控 | 旧版等待逻辑依赖文件/旧式流程，且没有可靠调试旁路 | 新版显式等待 `/ly/game/is_start`，并支持调试跳过和超时继续 | `sentary` 更适合比赛和离车调试双场景 |
| 导航策略 | 主要是固定点位逻辑，缺少 profile 语义 | 增加 `LeagueSimple`、`ShowcasePatrol`、`NaviDebugPlan` | `sentary` 已经把比赛、展示、导航调试分开 |
| 姿态决策 | 没有完整姿态状态机 | 新增 `SelectPosture`、`PostureLogic.cpp`、`PostureManager.cpp` | `sentary` 新增进攻/防御/移动姿态层 |
| 运行时保护 | 基本没有统一保护层 | 有 `RuntimeGuard`、`soft recovery`、`critical input stale` 安全降级 | `sentary` 更强调实机稳定性 |

---

## 2. 最关键的行为变化

### 2.1 决策执行方式

`ros2_ly_ws`：

- 虽然有 `main.xml`，但旧版 [GameLoop.cpp](../../../ros2_ly_ws/src/behavior_tree/src/GameLoop.cpp) 里实际是手动调用一串函数。
- 旧版更像“代码里硬编码流程，BT 只是留下来的接口层”。

`ros2_ly_ws_sentary`：

- 新版 [main.xml](../../src/behavior_tree/Scripts/main.xml) 已经明确完整调度顺序。
- 新版 [BehaviorTree.cpp](../../src/behavior_tree/src/BehaviorTree.cpp) 注册了完整节点集：
  - `UpdateGlobalData`
  - `SelectAimMode`
  - `SelectStrategyMode`
  - `ExecuteHitSentryStrategy`
  - `ExecuteHitHeroStrategy`
  - `ExecuteProtectedStrategy`
  - `ExecuteNaviTestStrategy`
  - `ExecuteLeagueSimpleStrategy`
  - `PreprocessData`
  - `SelectAimTarget`
  - `SelectPosture`
  - `PublishAll`

结论：

- 旧版是“函数式决策流”。
- 新版是“BT 驱动的决策框架”。

### 2.2 锁敌与火控判定

`ros2_ly_ws`：

- 主要用 `isFindTargetAtomic` 判断是否找到目标。
- 丢目标后会较长时间沿用旧角度，逻辑比较粗。
- 收到 `predictor/outpost/buff` 的目标后，基本就直接进入“可控/可打”路径。

`ros2_ly_ws_sentary`：

- `AimData` 新增：
  - `Valid`
  - `Fresh`
  - `HasLatchedAngles`
  - `LastValidTime`
- `PublishTogether()` 改成三段：
  - `has_fresh_target`：本帧有有效目标
  - `hold_last_lock`：短时丢帧时保留上次锁角
  - 否则回落到当前云台角/搜索逻辑
- `predictor` 回调里，“跟随”和“开火”被解耦：
  - 只要 yaw/pitch 有限值，就允许跟随
  - 是否开火看 `msg->status`

结论：

- 旧版是“找到目标就上”。
- 新版是“目标新鲜度、有效性、短时保持”三层控制，明显更稳。

### 2.3 开赛等待与调试旁路

`ros2_ly_ws`：

- `WaitBeforeGame()` 里以“云台角非 0”判断是否收到首帧云台数据，这个判断不可靠，因为 `0` 本身就是合法角度。
- 开赛等待不够体系化，调试旁路能力弱。

`ros2_ly_ws_sentary`：

- 用 `hasReceivedGimbalAngles_` 判断是否收到首帧云台消息。
- 等待 `/ly/game/is_start`。
- 新增 3 个重要 launch/ROS 参数：
  - `debug_bypass_is_start`
  - `wait_for_game_start_timeout_sec`
  - `league_referee_stale_timeout_ms`

结论：

- 新版把“比赛门控”和“调试绕过”正式参数化了。

### 2.4 导航决策的分层

`ros2_ly_ws`：

- 主要是：
  - `SetPositionHitHero`
  - `SetPositionHitSentry`
  - `SetPositionProtect`
  - `SetPositionNaviTest`
- 更像若干战术函数堆在一起。

`ros2_ly_ws_sentary`：

- 仍保留老函数，但外层多了 profile 和计划层：
  - `LeagueSimple`
  - `ShowcasePatrol`
  - `NaviDebugPlan`
- 且 recovery 判断更严格，支持裁判数据新鲜度检查。

结论：

- 新版导航决策比旧版多了一层“模式管理”和“数据有效性保护”。

### 2.5 姿态系统

`ros2_ly_ws`：

- 没有姿态决策层。
- 也没有 `/ly/control/posture` 和 `/ly/gimbal/posture` 这对链路。

`ros2_ly_ws_sentary`：

- 增加新 topic：
  - `/ly/control/posture`
  - `/ly/gimbal/posture`
- `behavior_tree` 增加：
  - `SelectPosture`
  - `PostureLogic.cpp`
  - `PostureManager.cpp`
- `gimbal_driver` 增加姿态下发与回读解析。

结论：

- 这是两个项目在“决策维度”上最大的结构性差异之一。

---

## 3. 重要参数变化总表

### 3.1 旧版已有、但新版语义增强的参数

| 参数 | `ros2_ly_ws` | `ros2_ly_ws_sentary` | 变化意义 |
| --- | --- | --- | --- |
| `AimDebug.StopFire` | 有 | 有 | 保留 |
| `AimDebug.StopRotate` | 有 | 有 | 保留 |
| `AimDebug.StopScan` | 有 | 有 | 保留 |
| `AimDebug.HitBuff` | 有 | 有 | 保留 |
| `AimDebug.HitOutpost` | 有 | 有 | 保留 |
| `AimDebug.HitCar` | 有 | 有 | 保留 |
| `Rate.FireRate` | 有 | 有 | 保留 |
| `Rate.TreeTickRate` | 有 | 有 | 保留 |
| `Rate.NaviCommandRate` | 有 | 有 | 保留 |
| `GameStrategy.HitBuff` | 有 | 有 | 保留 |
| `GameStrategy.HitOutpost` | 有 | 有 | 保留 |
| `GameStrategy.TestNavi` | 有 | 有 | 保留 |
| `GameStrategy.HitSentry` | 有 | 有 | 保留 |
| `GameStrategy.Protected` | 有 | 有 | 保留 |
| `NaviSetting.UseXY` | 有 | 有 | 但新版在不同 profile 下会被主动切成 `true/false` |
| `ScanCounter` | 有 | 有 | 保留 |

### 3.2 新版新增的重要参数

| 参数 | 默认值/示例值 | 所在文件 | 作用 |
| --- | --- | --- | --- |
| `AimDebug.FireRequireTargetStatus` | `true` | `config.json` / `regional_competition.json` | 控制“是否必须 `Target.status=true` 才允许开火” |
| `CompetitionProfile` | `regional` / `league` | `regional_competition.json` / `league_competition.json` | 选择比赛模式 |
| `LeagueStrategy.UseHealthRecovery` | `true` | `league_competition.json` | 联盟赛是否因低血回补 |
| `LeagueStrategy.HealthRecoveryThreshold` | `100` | `league_competition.json` | 联盟赛低血阈值 |
| `LeagueStrategy.UseAmmoRecovery` | `true` | `league_competition.json` | 联盟赛是否因低弹回补 |
| `LeagueStrategy.AmmoRecoveryThreshold` | `30` | `league_competition.json` | 联盟赛低弹阈值 |
| `LeagueStrategy.MainGoal` | `18` | `league_competition.json` | 联盟赛主占点 |
| `LeagueStrategy.PatrolGoals` | `[]` | `league_competition.json` | 联盟赛巡逻点列表 |
| `LeagueStrategy.GoalHoldSec` | `15` | `league_competition.json` | 联盟赛目标切换停留时间 |
| `ShowcasePatrol.Enable` | `true/false` | `showcase_competition.json` | 是否启用展示巡逻 |
| `ShowcasePatrol.Goals` | `[1,2,3,4]` 等 | `showcase_competition.json` | 展示巡逻点 |
| `ShowcasePatrol.GoalHoldSec` | `10` | `showcase_competition.json` | 展示巡逻停留时间 |
| `ShowcasePatrol.Random` | `false` | `showcase_competition.json` | 是否随机巡逻 |
| `ShowcasePatrol.DisableTeamOffset` | `true` | `showcase_competition.json` | 是否关闭蓝方点位偏移 |
| `ShowcasePatrol.IgnoreRecovery` | `true` | `showcase_competition.json` | 展示模式忽略回血/补弹回补 |
| `NaviDebug.Enable` | `true/false` | `navi_debug_competition.json` | 是否启用导航调试模式 |
| `NaviDebug.PlanFile` | `Scripts/ConfigJson/navi_debug_points.json` | `navi_debug_competition.json` | 导航计划文件 |
| `NaviDebug.ActivePlan` | `test_site_random` 等 | `navi_debug_competition.json` | 选中的计划名 |
| `NaviDebug.IgnoreRecovery` | `true` | `navi_debug_competition.json` | 导航调试时是否忽略回补 |
| `NaviDebug.DisableTeamOffset` | `true` | `navi_debug_competition.json` | 导航调试是否关偏移 |
| `NaviDebug.SpeedLevel` | `1` | `navi_debug_competition.json` | 导航速度等级 |

### 3.3 姿态系统参数

| 参数 | 默认值 | 含义 | 决策影响 |
| --- | --- | --- | --- |
| `Posture.Enable` | `true` | 总开关 | 是否启用姿态状态机 |
| `Posture.SwitchCooldownSec` | `5` | 切换冷却 | 限制频繁切姿态 |
| `Posture.MaxSinglePostureSec` | `180` | 单姿态累计上限 | 接近规则上限时会降档 |
| `Posture.EarlyRotateSec` | `165` | 提前轮换阈值 | 提前换姿态避免超时 |
| `Posture.MinHoldSec` | `10` | 最短保持时间 | 防抖 |
| `Posture.PendingAckTimeoutMs` | `600` | 等待回读超时 | 切姿态未确认时多久开始重试 |
| `Posture.RetryIntervalMs` | `300` | 重试间隔 | 姿态命令重发周期 |
| `Posture.MaxRetryCount` | `3` | 最大重试次数 | 超过则放弃 pending |
| `Posture.OptimisticAck` | `true` | 乐观确认 | 无回读时是否先认为切换成功 |
| `Posture.TargetKeepMs` | `800` | 目标保持窗口 | 目标短时丢失时仍保持攻击倾向 |
| `Posture.DamageKeepSec` | `4` | 受击保持窗口 | 受击后一段时间偏保守 |
| `Posture.DamageBurstWindowMs` | `0` 或 `1500` | 短时重受击统计窗口 | 展示模式里会触发更敏感的防御切换 |
| `Posture.DamageBurstThreshold` | `0` 或 `1` | 重受击阈值 | 达到后直接偏防御 |
| `Posture.DamageBurstDefenseHoldSec` | `0` 或 `5` | 短时重受击后防守保持 | 防止姿态来回抖动 |
| `Posture.LowHealthThreshold` | `150` | 低血阈值 | 更偏防御 |
| `Posture.VeryLowHealthThreshold` | `80` | 极低血阈值 | 进一步提高防守权重 |
| `Posture.LowAmmoThreshold` | `30` | 低弹阈值 | 降低攻击倾向 |
| `Posture.ScoreHysteresis` | `2` | 分差迟滞 | 新姿态分数领先不够时不切换 |

### 3.4 启动参数新增

| 参数 | 旧版 | 新版 | 作用 |
| --- | --- | --- | --- |
| `competition_profile` | 无 | 有 | 选择 `regional/league` |
| `bt_config_file` | 无 | 有 | 指定行为树 JSON 配置 |
| `bt_tree_file` | 无 | 有 | 指定行为树 XML |
| `debug_bypass_is_start` | 无 | 有 | 调试时跳过开赛门控 |
| `wait_for_game_start_timeout_sec` | 无 | 有 | 等待开赛超时后继续 |
| `league_referee_stale_timeout_ms` | 无 | 有 | 联盟赛回补逻辑的裁判数据新鲜度保护 |

---

## 4. 旧版配置文件和新版 profile 的对应关系

### 4.1 `ros2_ly_ws` 旧配置组织

旧版主要是这些文件：

- `Scripts/config.json`
- `Scripts/ConfigJson/hero.json`
- `Scripts/ConfigJson/hit_sentry.json`
- `Scripts/ConfigJson/navi_test.json`
- `Scripts/ConfigJson/protect.json`

它们本质上是“不同战术场景的几份静态配置”，但没有形成统一 profile 机制。

### 4.2 `ros2_ly_ws_sentary` 新配置组织

新版新增：

- `Scripts/ConfigJson/regional_competition.json`
- `Scripts/ConfigJson/league_competition.json`
- `Scripts/ConfigJson/showcase_competition.json`
- `Scripts/ConfigJson/navi_debug_competition.json`
- `Scripts/ConfigJson/navi_debug_points.json`

这意味着新版从“几份散装配置”升级成了“比赛模式 + 专项模式 + 计划文件”的结构。

---

## 5. 我认为最重要的 8 个差异

| 排名 | 差异 | 为什么重要 |
| --- | --- | --- |
| 1 | 新版 BT 真正接管主决策流程 | 这决定后续改策略时该改 XML+节点还是继续堆 if/else |
| 2 | 新版新增姿态状态机 | 这是旧版完全没有的一层控制能力 |
| 3 | 新版把比赛 profile 配置化 | 联盟赛、分区赛、展示模式已经不是同一套硬编码逻辑 |
| 4 | 新版目标跟随与开火解耦 | 实机会明显更稳，尤其在短时丢帧时 |
| 5 | 新版增加开赛门控旁路和超时参数 | 离车调试和正式比赛终于能共存 |
| 6 | 新版 recovery 逻辑增加裁判数据新鲜度判断 | 避免“没收到血量/弹量就误判为 0” |
| 7 | 新版导航策略新增 `LeagueSimple/Showcase/NaviDebug` | 说明项目已经从单一比赛逻辑扩展成多场景系统 |
| 8 | 新版新增运行时保护和安全发布 | 决策崩溃或输入超时不再直接裸奔 |

---

## 6. 维护建议

### 如果你后面只想改“比赛决策”

优先看这些文件：

- `src/behavior_tree/src/GameLoop.cpp`
- `src/behavior_tree/src/PostureLogic.cpp`
- `src/behavior_tree/src/PostureManager.cpp`
- `src/behavior_tree/Scripts/main.xml`
- `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
- `src/behavior_tree/Scripts/ConfigJson/league_competition.json`

### 如果你后面只想改“启动方式/调试方式”

优先看这些文件：

- `src/behavior_tree/launch/sentry_all.launch.py`
- `src/behavior_tree/launch/behavior_tree.launch.py`
- `scripts/start.sh`
- `scripts/debug.sh`

### 如果你后面只想改“姿态联调”

优先看这些文件：

- `src/behavior_tree/src/PostureLogic.cpp`
- `src/behavior_tree/src/PostureManager.cpp`
- `src/behavior_tree/src/PublishMessage.cpp`
- `src/behavior_tree/include/Topic.hpp`
- `src/gimbal_driver/main.cpp`

---

## 7. 一句话总结

`ros2_ly_ws` 的决策层还是“代码主导、BT 辅助”；`ros2_ly_ws_sentary` 已经是“BT 主导、配置分场景、姿态独立成层、运行时保护更完整”的下一阶段版本。
