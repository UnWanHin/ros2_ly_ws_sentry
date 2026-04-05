# 哨兵決策與輔瞄調參手冊

> 適用專案：`ros2_ly_ws_sentary`  
> 目的：讓接手者快速看懂「現在實際在跑的決策流程」與「輔瞄該去哪裡調參」。

---

## 1. 系統現況（先講結論）

目前系統的核心決策在 `behavior_tree`，已走 BT.CPP v4 主流程，並使用雙黑板：
- `GlobalBlackboard_`：跨 tick 持久資料（比賽狀態、血量、策略模式、AimMode 等）
- `TickBlackboard_`：單次 tick 中間資料（可打目標、可靠敵方位置、當前目標等）

說明：
- 聯盟賽與分區賽共用同一套雙黑板結構。
- 兩種賽制差異在策略分支與配置值，不在黑板架構。

決策輸出仍沿用原本業務資料與接口：
- 雲台控制：`/ly/control/angles`、`/ly/control/firecode`
- 模式使能：`/ly/aa/enable`、`/ly/ra/enable`、`/ly/outpost/enable`
- 目標類型：`/ly/bt/target`
- 導航：`/ly/navi/goal` 或 `/ly/navi/goal_pos`、`/ly/navi/speed_level`

接口名稱、消息類型、字段名保持不變。

---

## 2. 當前決策流程（behavior_tree）

### 2.1 主樹流程

`src/behavior_tree/Scripts/main.xml` 的主樹是：

1. `UpdateGlobalData`
2. `SelectAimMode`
3. `SelectStrategyMode`
4. `SubTree: StrategyDispatch`
5. `PreprocessData`
6. `SelectAimTarget`
7. `SelectPosture`
8. `PublishAll`

對應關係：
- `UpdateGlobalData`：重置 Tick 黑板 + 把 Application 狀態寫入 Global 黑板
- `SelectAimMode`：沿用原 `SetAimMode()` + `CheckDebug()`
- `SelectStrategyMode`：根據比賽狀態做策略模式動態切換
- `StrategyDispatch`：五個策略子樹分開執行（HitSentry / HitHero / LeagueSimple / Protected / NaviTest）
- `PreprocessData`：沿用 `ProcessData()`
- `SelectAimTarget`：沿用 `SetAimTarget()`
- `SelectPosture`：根據策略/目標/資源狀態選姿態
- `PublishAll`：沿用 `PublishTogether()` + `PrintMessageAll()`

### 2.2 策略子樹與動態切換

策略切換節點：`SelectStrategyModeNode`（`src/behavior_tree/include/BTNodes.hpp`）

核心規則（不改原業務意圖）：
- 若 `competition_profile=league`：直接切 `LeagueSimple`
- 開局前 10 秒：保持 `config.json` 初始策略
- 低資源條件：切 `Protected`
  - `SelfHealth < 100` 或 `TimeLeft <= 120` 或剩餘能量低
- 若當前是 `NaviTest` 且 `now_time < 340`：保持 `NaviTest`
- 若滿足哨兵窗口：切 `HitSentry`
  - `EnemyOutpostHealth > 0 && SelfOutpostHealth > 100 && now_time < 55`
- 否則：切 `HitHero`

策略子樹執行時，最終仍調用既有函數：
- `ExecuteHitSentryStrategy` -> `SetPositionHitSentry()`
- `ExecuteHitHeroStrategy` -> `SetPositionHitHero()`
- `ExecuteLeagueSimpleStrategy` -> `SetPositionLeagueSimple()`
- `ExecuteProtectedStrategy` -> `SetPositionProtect()`
- `ExecuteNaviTestStrategy` -> `SetPositionNaviTest()`

### 2.3 聯盟賽現在怎麼決策

聯盟賽現在不是把原本分區賽邏輯「砍半」後硬套，而是單獨走 `LeagueSimple`：

- 缺省導航模式：`NaviSetting.UseXY=false`
- BT 只發目標序號：`/ly/navi/goal`
- 當前占點序號：`OccupyArea = 18`
- 規則：
  - `myselfHealth < HealthRecoveryThreshold` 時回 `Recovery`
  - `ammoLeft <= AmmoRecoveryThreshold` 時也可回 `Recovery`
  - 以上判斷只在對應裁判數據「已收到（可選：未過期）」時生效，避免默認 `0` 誤判
  - 其他時間固定去 `OccupyArea`

`PatrolGoals` 仍保留在配置裡，之後如果你想把聯盟賽擴成「多個占點/巡邏點輪換」，只要往裡加序號即可；目前默認是空，等於只跑一個占點。

### 2.4 當前「調試模式」邊界（重要）

- `behavior_tree` 啟動後，會先 `WaitBeforeGame()`，再在 `WaitForGameStart()` 等待 `/ly/game/is_start=true`。
- 新增兩個可選調試參數（默認關閉，不影響比賽路徑）：
  - `debug_bypass_is_start:=true`：直接跳過 `is_start` 門控
  - `wait_for_game_start_timeout_sec:=N`：等待 N 秒後跳過門控
- 便捷入口：`./scripts/start.sh nogate --mode regional`（固定注入 `debug_bypass_is_start:=true`）
- `Application::CheckDebug()` 只覆蓋 `AimMode`（例如強制 Buff / Outpost），不會跳過開賽等待。
- `detector_config.debug_mode/debug_team_blue` 只影響 detector 的敵我顏色過濾，不是全鏈路總開關。
- 離車環境無穩定裁判數據時，建議使用 `detector + mapper_node` 做火控聯調。

---

## 3. `SetPosition` 現在是什麼狀態

### 3.1 為什麼 `SetPosition.cpp` 被註釋

`src/behavior_tree/src/SetPosition.cpp` 目前是「歷史備份版本」，整檔註釋保留，目的是：
- 保留舊策略邏輯對照，不硬刪有參考價值代碼
- 避免和現行實作重複定義同名函數

### 3.2 真正在跑的 SetPosition 在哪裡

現行有效函數在 `src/behavior_tree/src/GameLoop.cpp`：
- `SetPositionRepeat()`
- `SetPositionProtect()`
- `SetPositionNaviTest()`
- `SetPositionHitSentry()`
- `SetPositionHitHero()`
- `SetPositionLeagueSimple()`
- `CheckPositionRecovery()`

BT 節點實際調用的就是這些函數。

---

## 4. `GameLoop.cpp` 在做什麼

`GameLoop()` 是每個循環 tick 的執行入口，主要做三件事：

1. 初始化黑板
- 建立 Global/Tick Blackboard
- 寫入初始鍵值（如 `LastCommandTime`、`CommandInterval`、`AimTarget`）

2. 主循環
- `rclcpp::spin_some(node_)` 收 ROS 訂閱資料
- `TreeTick()` 執行 BT（`tickWhileRunning`）
- `treeTickRateClock.sleep()` 控頻

3. 由 BT 末端節點 `PublishAll` 完成消息發布
- `PublishTogether()` 統一雲台/火控/掃描/旋轉邏輯
- `PublishMessageAll()` 發所有控制 topic

---

## 5. 輔瞄調參在哪裡改

### 結論
是，主入口仍是 `detector` 的 YAML。

但要分兩種啟動方式看：
- 直接 `ros2 launch behavior_tree sentry_all.launch.py`：
  - 默認讀分層配置（`base + detector/predictor/outpost/buff + override`）
- `./scripts/start.sh gated`：
  - 默認注入 `scripts/config/stack/*.yaml` 分層配置
  - 僅注入模式相關參數（`competition_profile` / `bt_config_file` / `offline`）
  - 需要時可手動傳 `config_file:=...` 做全局覆蓋

同一份參數會被多個節點共用（detector / predictor / gimbal_driver 等 launch 都載這份）。

### 5.1 常用調參區塊

1. 相機輸入
- `camera_param/ExposureTime`
- `camera_param/Gain`
- `camera_param/RedBalanceRatio`
- `camera_param/GreenBalanceRatio`
- `camera_param/BlueBalanceRatio`
- `camera_param/camera_sn`

2. detector 模型與模式
- `detector_config/detector_path`
- `detector_config/car_model_path`
- `detector_config/classifier_path`
- `detector_config/debug_mode`
- `detector_config/debug_team_blue`
- `detector_config/use_video`
- `detector_config/use_ros_bag`

補充：`debug_mode/debug_team_blue` 僅影響 detector 顏色過濾；不控制 behavior_tree 的啟停或開賽等待。

3. PnP / 外參（影響角度解算）
- `solver_config/camera_offset`
- `solver_config/camera_rotation`
- `solver_config/camera_intrinsic_matrix`
- `solver_config/camera_distortion_coefficients`

4. 彈道與補償（影響打點高低/左右）
- `controller_config/bullet_speed`
- `controller_config/shoot_delay`
- `controller_config/shoot_table_adjust.*`

### 5.2 決策策略調參（不是 detector）

`behavior_tree` 的策略初值與調試開關在：
- `src/behavior_tree/Scripts/config.json`
- `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
- `src/behavior_tree/Scripts/ConfigJson/league_competition.json`
- `src/behavior_tree/Scripts/ConfigJson/showcase_competition.json`
- `src/behavior_tree/Scripts/ConfigJson/navi_debug_competition.json`
- `src/behavior_tree/Scripts/ConfigJson/navi_debug_points.json`

常改項：
- `GameStrategy.HitSentry / TestNavi / Protected / HitBuff / HitOutpost`（初始策略）
- `AimDebug.StopFire / StopRotate / StopScan / HitBuff / HitOutpost`
- `Rate.FireRate / TreeTickRate / NaviCommandRate`
- `CompetitionProfile`
- `LeagueStrategy.UseHealthRecovery / HealthRecoveryThreshold / UseAmmoRecovery / AmmoRecoveryThreshold / MainGoal / PatrolGoals / GoalHoldSec`
- `ShowcasePatrol.Enable / Goals / GoalHoldSec / Random / DisableTeamOffset`
- `ShowcasePatrol.IgnoreRecovery`
- `NaviDebug.Enable / PlanFile / ActivePlan / IgnoreRecovery / DisableTeamOffset / SpeedLevel`
- `Posture.DamageBurstWindowMs / DamageBurstThreshold / DamageBurstDefenseHoldSec`
- `debug_bypass_is_start / wait_for_game_start_timeout_sec / league_referee_stale_timeout_ms`（ROS 參數，默認關閉）

當前聯盟賽推薦值：
- `CompetitionProfile = "league"`
- `NaviSetting.UseXY = false`
- `LeagueStrategy.MainGoal = 3`（`OccupyArea`）
- `LeagueStrategy.PatrolGoals = []`

當前展示模式補充：
- `src/behavior_tree/Scripts/ConfigJson/showcase_competition.json`
- `ShowcasePatrol.Goals = [6, 11, 14, 15]` 可直接改成你現場要演示的點位序列
- `ShowcasePatrol.DisableTeamOffset = true` 時，`/ly/navi/goal` 直接發基礎 ID `0..18`
- `ShowcasePatrol.IgnoreRecovery = true` 時，展示模式可在無裁判輸入時直接巡邏

當前導航調試模式補充：
- `src/behavior_tree/Scripts/ConfigJson/navi_debug_competition.json`
- `src/behavior_tree/Scripts/ConfigJson/navi_debug_points.json`
- `NaviDebug.ActivePlan` 或點位文件內的 `ActivePlan` 可切換臨時路線
- `Plans.<name>.Mode = random|sequence`
- `Plans.<name>.Goals = [...]` 只改序號列表，不改老點位定義

---

## 6. 快速鏈路自檢（上車前）

### 6.1 核心 topic 存在性

```bash
ros2 topic list | rg '^/ly/'
```

至少要看到（節選）：
- `/ly/gimbal/angles`
- `/ly/predictor/target`
- `/ly/outpost/target`
- `/ly/buff/target`
- `/ly/control/angles`
- `/ly/control/firecode`
- `/ly/aa/enable`
- `/ly/ra/enable`
- `/ly/outpost/enable`
- `/ly/bt/target`

### 6.2 方向檢查（誰發誰收）

```bash
ros2 topic info /ly/bt/target -v
ros2 topic info /ly/aa/enable -v
ros2 topic info /ly/control/angles -v
ros2 topic info /ly/predictor/target -v
```

預期：
- `/ly/bt/target`：`behavior_tree` 發，`detector/predictor` 收
- `/ly/aa/enable`：`behavior_tree` 發，`detector` 收
- `/ly/control/angles`：`behavior_tree` 發，`gimbal_driver` 收
- `/ly/predictor/target`：`predictor` 發，`behavior_tree` 收

### 6.3 頻率檢查

```bash
ros2 topic hz /ly/control/angles
ros2 topic hz /ly/control/firecode
ros2 topic hz /ly/detector/armors
```

若 `control` 類 topic 不出數據，優先查：
- `behavior_tree` 是否成功載入 `Scripts/main.xml`
- 比賽開始旗標 `/ly/game/is_start` 是否已進入主循環
- `behavior_tree` 日誌中是否有 BT tick 失敗

---

## 7. 交接建議（不改接口前提）

1. 先在 `auto_aim_config.yaml` 做 detector/predictor 調參，再動策略。  
2. 策略只在 `config.json` 與 BT 條件閾值調，topic 接口不改名。  
3. `SetPosition.cpp` 維持註釋備份狀態，現行邏輯看 `GameLoop.cpp`。  
4. 用 `behavior_tree_trace.fbl` + Groot2 做離線分析，不依賴付費 Runtime Monitor。

---

## 8. 一鍵啟動建議

已提供總啟動入口（推薦）：
- `ros2 launch behavior_tree sentry_all.launch.py`

也提供工作區快捷腳本：
- `./scripts/start.sh gated`

腳本模式選擇（推薦）：
- 交互選擇（1=league, 2=regional, 3=showcase）
  - `./scripts/start.sh gated`
- 非交互聯盟賽
  - `./scripts/start.sh gated --mode league`
- 非交互分區賽
  - `./scripts/start.sh gated --mode regional`
- 非交互展示模式
  - `./scripts/start.sh showcase`

### 8.1 默認行為

默認會啟動：
- `gimbal_driver`
- `detector`
- `tracker_solver`
- `predictor`
- `outpost_hitter`
- `buff_hitter`
- `behavior_tree`

快速火控測試請另起終端按需啟動 `mapper_node` 或 `fire_flip_test`，默認不隨 `sentry_all` 自動啟動。

示例（mapper 火控聯調）：
```bash
ros2 run detector mapper_node \
  --target-priority 6,3,4,5 \
  --target-id 6 \
  --publish-team false \
  --enable-fire true \
  --auto-fire true
```

### 8.2 常用參數

1. 只測感知鏈路（不啟 BT）
```bash
ros2 launch behavior_tree sentry_all.launch.py use_behavior_tree:=false
```

2. 關閉打符或前哨節點
```bash
ros2 launch behavior_tree sentry_all.launch.py use_buff:=false use_outpost:=false
```

3. 指定參數檔
```bash
ros2 launch behavior_tree sentry_all.launch.py \
  config_file:=/home/unwanhin/ros2_ly_ws_sentary/src/detector/config/auto_aim_config.yaml
```

4. 切到聯盟賽 profile
```bash
ros2 launch behavior_tree sentry_all.launch.py \
  competition_profile:=league
```

5. 顯式指定 BT 比賽配置
```bash
ros2 launch behavior_tree sentry_all.launch.py \
  bt_config_file:=Scripts/ConfigJson/league_competition.json
```

---

## 9. 鏈路對齊檢查（2026-03-12）

本次對齊檢查結論：
- `scripts/start.sh gated` / `scripts/start.sh nogate` -> `sentry_all.launch.py` -> `behavior_tree` 的參數鏈路已對齊
  - `--mode`：可選 `1/2/3` 或顯式傳入 `league/regional/showcase`
  - `competition_profile`：由啟動模式自動對齊；其中 `mode 3` 仍映射到 `regional`
  - `bt_config_file`：按啟動模式自動對齊（`league/regional/showcase`）
  - `config_file`：默認沿用 launch 默認 YAML，僅在顯式傳入時覆蓋
- BT 層策略鏈路已對齊
  - `league`：固定走 `LeagueSimple`
  - `regional`：保持原分區賽策略切換
  - `showcase`：沿用 `regional` 主流程，但使用展示專用姿態參數
- 導航發布鏈路已對齊
  - `UseXY=false` -> 發 `/ly/navi/goal`
  - `UseXY=true` -> 發 `/ly/navi/goal_pos`

驗證記錄：
- `./scripts/selfcheck.sh sentry --static-only`：通過（PASS 36 / WARN 0 / FAIL 0）
- `colcon build --packages-select behavior_tree`：通過
- `./scripts/selfcheck.sh sentry --runtime-only --launch --wait 5 --skip-hz`：
  - 在當前沙箱環境受 DDS 權限限制（`RTPS_TRANSPORT_SHM/UDP permission denied`）未通過
  - 屬於環境限制，不是策略鏈路配置錯配
