# 哨兵展示 Runbook

## 1. 目标

本展示文档只覆盖两件事：

1. 辅瞄链路单独演示
2. 姿态切换链路演示

当前实现不新开第三套核心赛制逻辑。展示模式仍走 `regional` 主流程，只是启动时自动切到展示专用配置。

导航输出补充：

- 展示模式当前固定使用 `/ly/navi/goal`
- 对应配置里 `NaviSetting.UseXY = false`
- `/ly/navi/goal_pos` 仍留给后续分区赛链路继续对接

---

## 2. 快速入口

### 2.1 整车展示/调试模式

推荐直接用专用脚本：

```bash
./scripts/start.sh showcase
```

默认行为：

- 固定 `--mode 3 --no-prompt`
- 默认走 `showcase.launch.py`，并注入 `debug_bypass_is_start:=true`
- 自动注入展示配置 `Scripts/ConfigJson/showcase_competition.json`

如果现场已经接裁判系统，想保留开赛门控：

```bash
./scripts/start.sh showcase --with-gate
```

如果你想把“辅瞄控制输出”和“姿态切换”都放进裁判系统门控里，推荐直接跑整车展示链路，而不是单独跑辅瞄脚本：

```bash
./scripts/start.sh showcase --with-gate -- use_buff:=false use_outpost:=false
```

说明：

- `detector / tracker_solver / predictor` 会先启动
- `behavior_tree` 会等待 `/ly/game/is_start=true`
- 真正接管 `/ly/control/*` 与姿态输出，要等开赛门控放开后才开始

如果是离线/回放：

```bash
./scripts/start.sh showcase --offline
```

### 2.2 辅瞄单独跑

最简单的单独辅瞄入口：

```bash
./scripts/debug.sh autoaim-test
```

这条命令实际等价于：

```bash
./scripts/debug.sh autoaim-debug --mode fire --offline
```

它会拉起：

- `detector`
- `tracker_solver`
- `predictor`
- `mapper_node`

并直接写：

- `/ly/control/angles`
- `/ly/control/firecode`

所以单独跑辅瞄时，不要同时跑 `behavior_tree`。否则会出现多控制源冲突。

补充：

- 这条 standalone 辅瞄链路不经过裁判系统门控
- 因为它走的是 `auto_aim.launch.py`，里面没有 `WaitForGameStart()`
- 如果你要求“也要挂在开赛门控后再出控制”，不要用这条 standalone 入口，改走上面的整车展示链路

---

## 3. 辅瞄单独跑怎么选

### 3.1 只看识别和可视化，不接管控制

```bash
./scripts/debug.sh autoaim-debug --mode perception --online
```

用途：

- 只看感知链和可视化
- 不启动 `mapper_node`
- 不抢 `/ly/control/*`

适合展示“小装甲识别画面”。

### 3.2 要看完整辅瞄链并直接控云台

```bash
./scripts/debug.sh autoaim-test --online
```

或：

```bash
./scripts/debug.sh autoaim-debug --mode fire --online
```

用途：

- 感知链 + `mapper_node`
- 直接输出控制角度和火控

适合展示“发现目标 -> 给角度 -> 跟踪/开火”。

### 3.3 已经有感知链，只想单独起 mapper

```bash
./scripts/debug.sh autoaim-debug --mode mapper --online
```

适合拆链调试，不是现场优先方案。

---

## 4. 姿态展示当前逻辑

展示模式配置文件：

- `src/behavior_tree/Scripts/ConfigJson/showcase_competition.json`

展示巡逻点也在这份文件里改：

- `ShowcasePatrol.Enable`：是否启用展示专用巡逻
- `ShowcasePatrol.Goals`：展示巡逻点位 ID 列表
- `ShowcasePatrol.GoalHoldSec`：每个点停留秒数
- `ShowcasePatrol.Random`：`true`=随机切点，`false`=按数组顺序轮巡
- `ShowcasePatrol.DisableTeamOffset`：`true` 时 `/ly/navi/goal` 直接发基础 ID，不再对蓝方加 `+50`
- `ShowcasePatrol.IgnoreRecovery`：`true` 时忽略回血/补弹回补逻辑，适合无裁判系统的展示场景

顺序说明：

- 当 `ShowcasePatrol.Random = false` 时，巡逻顺序就是 `ShowcasePatrol.Goals` 里数组从左到右的顺序
- 当 `ShowcasePatrol.Random = true` 时，仍然只会从 `ShowcasePatrol.Goals` 这组点里选，但按随机方式切换

当前默认示例：

```json
"ShowcasePatrol": {
  "Enable": true,
  "Goals": [6, 11, 14, 15],
  "GoalHoldSec": 5,
  "Random": true,
  "DisableTeamOffset": true,
  "IgnoreRecovery": true
}
```

当前展示导航链路：

- `UseXY = false`
- 发布 `/ly/navi/goal`
- 不发布 `/ly/navi/goal_pos`
- `DisableTeamOffset = true` 时，展示模式下发的是基础点位 ID（`0..18`）
- `IgnoreRecovery = true` 时，展示模式不会因为缺少血量/弹药回传而先切去 `Recovery`

当前展示入口能力：

- `./scripts/start.sh showcase` 默认会起整套展示主链
- 有普通辅瞄
- 有姿态展示
- 有展示巡逻
- 默认配置下会直接进入展示巡逻，不依赖裁判回血/弹药回传
- 小陀螺不属于“独立强制常开模式”，是否输出 Rotate 仍由 `behavior_tree` 当前逻辑决定
- 如果你只想单独测小陀螺，请改用 `./scripts/debug.sh chassis-gyro`

点位 ID 对照：

- `0 Home`
- `1 Base`
- `2 Recovery`
- `3 BuffShoot`
- `4 LeftHighLand`
- `5 CastleLeft`
- `6 Castle`
- `7 CastleRight1`
- `8 CastleRight2`
- `9 FlyRoad`
- `10 OutpostArea`
- `11 MidShoot`
- `12 LeftShoot`
- `13 OutpostShoot`
- `14 BuffAround1`
- `15 BuffAround2`
- `16 RightShoot`
- `17 HoleRoad`
- `18 OccupyArea`

补充：

- 这套展示巡逻当前是“改 `/ly/navi/goal` 的 ID 序列”，不是新建任意 XY 点。
- 如果后面你想在非比赛场地上跑“全新临时坐标”，还得让导航侧也支持新的点表或重新接回 `/ly/navi/goal_pos`。

当前姿态逻辑是：

1. 默认无目标时优先 `Move`
2. 有目标或短时保目标时切 `Attack`
3. 短时间内累计掉血超过阈值时直接切 `Defense`
4. 防守保持时间过后，且脱战后，再回主循环

当前展示参数：

- `TargetKeepMs = 400`
- `DamageBurstWindowMs = 1200`
- `DamageBurstThreshold = 25`
- `DamageBurstDefenseHoldSec = 3`
- `MinHoldSec = 1`
- `SwitchCooldownSec = 1`

对应效果：

- 目标一出现，姿态切换更快
- 短时间被打掉一小段血，就能展示“进攻 -> 防守”
- 防守停留不会太久，便于现场重复演示

---

## 5. 现场怎么喂数据

### 5.1 `Move -> Attack`

这条链只需要有目标输入。

来源通常是：

- `/ly/predictor/target`
- `/ly/outpost/target`
- `/ly/buff/target`

对展示来说，最常见的是普通辅瞄目标，即 `/ly/predictor/target`。

### 5.2 `Attack -> Defense`

这条链现在依赖“血量下降”。

也就是：

- `myselfHealth` 必须有回传
- 且在 `1200ms` 窗口内累计掉血达到 `25`

因此：

- 接裁判系统时，可以直接展示
- 不接裁判系统时，这条分支不会自然触发

如果现场没有裁判系统，当前代码下建议：

1. 先展示 `Move -> Attack`
2. 如必须展示 `Defense`，需要额外提供 HP 模拟输入

当前仓库里还没有单独的“假受击注入脚本”。

---

## 6. 推荐展示流程

### 6.1 项目一：辅瞄可视化

推荐命令：

```bash
./scripts/debug.sh autoaim-debug --mode perception --online
```

如果还要直接控云台：

```bash
./scripts/debug.sh autoaim-test --online
```

展示点：

- 识别框和目标切换
- 小装甲跟踪稳定性
- 目标进入后角度控制输出

如果现场明确要求“通过裁判系统开赛后才开始控制”，改用：

```bash
./scripts/start.sh showcase --with-gate -- use_buff:=false use_outpost:=false
```

这时展示上看到的是：

- 感知链先在线
- `/ly/game/is_start=true` 前，行为树不接管控制
- 开赛后才开始正式输出姿态与控制

### 6.2 项目二：姿态切换

推荐命令：

```bash
./scripts/start.sh showcase
```

推荐讲解顺序：

1. 初始巡逻/移动，姿态为 `Move`
2. 人为给目标，让系统切到 `Attack`
3. 若有裁判血量输入，短时受击触发 `Defense`
4. 脱战后回到主循环

---

## 7. 快速检查项

整车展示时建议看：

```bash
ros2 topic echo /ly/control/posture
ros2 topic echo /ly/gimbal/posture
```

辅瞄单跑时建议看：

```bash
ros2 topic echo /ly/control/angles
ros2 topic echo /ly/control/firecode
```

如果只演示识别，不控云台，则重点看 detector 的可视化窗口和 `/ly/predictor/target` 是否连续输出。

---

## 8. 当前边界

- `showcase` 只是启动模式，不是新的核心 `CompetitionProfile`
- 辅瞄单跑和整车展示不要同时抢 `/ly/control/angles`、`/ly/control/firecode`
- 无裁判系统时，`Defense` 展示依赖额外 HP 输入，当前仓库未提供注入脚本
