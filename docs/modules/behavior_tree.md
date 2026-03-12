# behavior_tree — 決策框架節點

## 概述

`behavior_tree` 是整個系統的**大腦/決策中心**，負責：
1. 匯總所有感知數據（来自 `gimbal_driver`：比賽狀態、血量；来自 `predictor/outpost_hitter/buff_hitter`：瞄準指令）
2. 運行決策邏輯（BehaviorTree.CPP v4 行為樹 + 姿態決策節點）
3. 向 `gimbal_driver` 發送最終的雲台控制指令、開火碼、導航指令

> 当前状态：主循環使用 `rclcpp::ok()`，並在每輪執行 `BTree.tickWhileRunning(...)`。

---

## 目錄結構

```
behavior_tree/
├── CMakeLists.txt
├── package.xml
├── main.cpp                    # 入口，創建 Application 並 Run()
├── Scripts/
│   ├── main.xml                # BT v4 行為樹主文件
│   ├── config.json             # 默認/兼容策略配置文件
│   └── ConfigJson/             # 可切換比賽 profile 配置
│       ├── regional_competition.json
│       └── league_competition.json
├── include/
│   ├── Application.hpp         # 核心類：所有狀態變量 + 所有函數聲明
│   ├── Node.hpp                # BT節點定義（BT v4 動作節點/條件節點等）
│   ├── Topic.hpp               # ROS Topic 定義（LY_DEF_ROS_TOPIC宏）
│   └── Robot.hpp               # Robot類/UnitType/UnitTeam 等遊戲數據類型
├── src/
│   ├── Application.cpp         # 構造函數、Run() 流程
│   ├── GameLoop.cpp            # 主循環邏輯（UpdateBlackBoard/TreeTick/PublishTogether）
│   ├── BehaviorTree.cpp        # BT初始化（RegisterTreeNodes, LoadBehaviorTree）
│   ├── Configuration.cpp       # 讀取 config.json
│   ├── Logger.cpp              # 日誌初始化
│   ├── PublishMessage.cpp      # 所有發布函數（PubGimbalControlData等）
│   ├── SetPosition.cpp         # 導航位置決策
│   ├── SubscribeMessage.cpp    # 所有訂閱回調（接收感知和遊戲數據）
│   └── WaitBeforeGame.cpp      # 等待比賽開始邏輯
├── module/                     # 工具庫
│   ├── BasicTypes.hpp          # 遊戲數據類型（AimMode、ArmorType、FireCodeType等）
│   ├── ROSTools.hpp            # ROSNode工具
│   ├── Area.hpp                # 地圖區域定義（各個點位的座標）
│   ├── Rate.hpp                # 頻率控制（RateClock、TimerClock）
│   ├── SineWave.hpp            # 正弦波生成（掃描時俯仰角波動）
│   ├── Counter.hpp             # 計數器
│   ├── Random.hpp              # 隨機數生成器
│   └── json.hpp                # nlohmann/json（第三方，953KB）
└── Logger/                     # 日誌系統（第三方或自實現）
    └── （多個頭文件和src）
```

---

## BT v4 依赖策略（2026-03）

- `CMake` 固定從工作區 `third_party/behaviortree_cpp_v4/install` 查找 BT v4：
  - `src/behavior_tree/CMakeLists.txt:18`
- `find_package(behaviortree_cpp ... NO_DEFAULT_PATH)`，禁用系統路徑回退：
  - `src/behavior_tree/CMakeLists.txt:39`
- 強制版本檢查 `>= 4.0.0`：
  - `src/behavior_tree/CMakeLists.txt:44`
- 可執行檔 RPATH 固定到 pinned BT 路徑：
  - `src/behavior_tree/CMakeLists.txt:90`
- `package.xml` 不再聲明 `behaviortree_cpp` 直接 `<depend>`，由 CMake pinned 路徑控制鏈接來源：
  - `src/behavior_tree/package.xml:1`

---

## 核心文件詳解

### `main.cpp` — 入口

```cpp
rclcpp::init(argc, argv);
BehaviorTree::Application app(argc, argv);
app.Run();
rclcpp::shutdown();
```

### `include/Application.hpp` + `src/Application.cpp` — 核心應用類

#### 成員變量（狀態存儲）

**感知數據（從回調函數更新）**：
| 變量 | 類型 | 數據來源 Topic |
|------|------|----------------|
| `team` | `UnitTeam` | `/ly/me/is_team_red` |
| `enemyOutpostHealth` | `uint16_t` | `/ly/enemy/op_hp` |
| `selfOutpostHealth` | `uint16_t` | `/ly/me/op_hp` |
| `selfBaseHealth` | `uint16_t` | `/ly/me/base_hp` |
| `ammoLeft` | `uint16_t` | `/ly/me/ammo_left` |
| `timeLeft` | `uint16_t` | `/ly/game/time_left` |
| `myselfHealth` | `uint16_t` | `/ly/me/hp` → sentry 血量 |
| `friendRobots` | `Robots` | `/ly/position/data`, `/ly/me/hp` |
| `enemyRobots` | `Robots` | `/ly/enemy/hp`, `/ly/position/data` |
| `teamBuff` | `BuffType` | `/ly/team/buff` |
| `armorList` | `array<ArmorData,10>` | `/ly/predictor/target` 等（含距離和類型） |
| `gimbalAngles` | `GimbalAnglesType` | `/ly/gimbal/angles` |
| `naviVelocity` | `VelocityType` | `/ly/gimbal/vel`（底盤速度反饋） |
| `isFindTargetAtomic` | `atomic<bool>` | 每次收到有效 `Target.msg`（status=true）時置 true |

**決策輸出數據**：
| 變量 | 說明 |
|------|------|
| `aimMode` | 當前瞄準模式（AutoAim/Buff/Outpost/RotateScan） |
| `targetArmor` | 要打的裝甲板類型 + 距離 |
| `autoAimData` | 普通瞄準的角度數據（來自 predictor） |
| `buffAimData` | 打符的角度數據（來自 buff_hitter） |
| `outpostAimData` | 前哨站角度數據（來自 outpost_hitter） |
| `gimbalControlData` | 最終發出的雲台控制數據（角度+火控） |
| `naviCommandGoal` | 導航目標點位（uint8，對應 Area 枚舉） |
| `speedLevel` | 底盤速度等級（0=停、1=正常、2=快） |

#### 構造流程（`Application.cpp`）

```
Application::Application()
├── rclcpp::init（如果還沒init）
├── create_node("behavior_tree")，允許從 launch 注入參數覆蓋
├── InitLogger()              → 初始化日誌（Logger子系統）
├── 從 ament_index 獲取包路徑 → 定位 Scripts/main.xml 和 config.json
├── 可選讀取 `competition_profile` / `bt_config_file` / `bt_tree_file`
├── SubscribeMessageAll()     → 訂閱全部上游 Topics
├── PublishMessageAll()       → 創建全部發布者
├── ConfigurationInit()       → 讀取 config.json
├── RegisterTreeNodes()       → 向 BT Factory 注冊自定義節點
└── LoadBehaviorTree()        → 從 main.xml 創建行為樹
```

#### 運行流程（`Application.cpp` → `Run()`）

```
Run()
├── WaitBeforeGame()  ← 等待比賽開始（阻塞，監聽 /ly/game/is_start）
├── gameStartTime = now()
└── GameLoop()        ← 主循環
```

---

### `src/GameLoop.cpp` — 主循環邏輯（最重要的文件）

#### `GameLoop()` — 主循環

```cpp
while (rclcpp::ok()) {
    rclcpp::spin_some(node_);                 // 處理回調
    const auto status = BTree.tickWhileRunning(std::chrono::milliseconds(1));
    treeTickRateClock.sleep();                // 頻率控制
}
```

#### `UpdateBlackBoard()` — 把感知數據寫黑板

把所有感知變量寫入 `BT::Blackboard`：
```cpp
BlackBoard->set<UnitTeam>("MyTeam", team);
BlackBoard->set<uint16_t>("TimeLeft", timeLeft);
BlackBoard->set<uint16_t>("SelfHealth", SelfHealth);
BlackBoard->set<uint16_t>("AmmoLeft", ammoLeft);
BlackBoard->set<Robots>("FriendRobots", friendRobots);
BlackBoard->set<Robots>("EnemyRobots", enemyRobots);
BlackBoard->set<uint16_t>("EnemyOutpostHealth", enemyOutpostHealth);
BlackBoard->set<uint16_t>("SelfOutpostHealth", selfOutpostHealth);
BlackBoard->set("ArmorList", armorList);
BlackBoard->set("TeamBuff", teamBuff);
```

#### `TreeTick()` — BT 根節點調度

```cpp
void TreeTick() {
    if (BTree.subtrees.empty()) return;
    const auto status = BTree.tickWhileRunning(std::chrono::milliseconds(1));
    if (status == BT::NodeStatus::FAILURE) { ... }
}
```

#### `SetAimMode()` — 模式切換邏輯

| 條件 | 設置 |
|------|------|
| config.HitBuff=true 且 now_time<25 且 buff次數≤15 | `AimMode::Buff` |
| Buff激活（DefenceBuff或VulnerabilityBuff>20） | `AimMode::RotateScan`（Buff進了就切回掃描） |
| config.HitOutpost=true 且 enemyOutpostHealth>0 且 now_time<90 | `AimMode::Outpost` |
| 其他 | `AimMode::RotateScan` |

#### `SetAimTarget()` — 目標選擇優先級

在普通模式（RotateScan）下，優先級：**英雄 > 步兵1/2（距離近的）> 哨兵 > 工程**

工程師在比賽開始60秒內不打（有無敵保護）。

#### `PublishTogether()` — 匯總並發布（最複雜的函數）

這個函數決定最終發出什麼角度和火控碼：

1. **小陀螺控制**：根據血量下降速度（`healthDecreaseDetector`）和底盤速度（`naviVelocity`），動態設置 `FireCode.Rotate`（0=停止、1-3=不同速度）
2. **有目標時**：
   - 按 `aimMode` 從對應的 `Aim*Data` 取角度
   - 觸發開火（`fireRateClock` 控製開火頻率）  
   - 打符模式下只在 `buffAimData.FireStatus=true` 時翻轉開火狀態
3. **無目標時（超過2秒）**：切換到掃描模式，Yaw+3° 偏移，Pitch 用 `SineWave` 做俯仰波動
4. 調用 `PublishMessageAll()` 發出最終指令

---

### `src/SubscribeMessage.cpp` — 所有訂閱回調

| 訂閱 Topic | 更新的變量 | 說明 |
|-----------|-----------|------|
| `/ly/me/is_team_red` | `team` | 我方顏色 |
| `/ly/game/is_start` | `is_game_begin` | 比賽開始標誌 |
| `/ly/game/time_left` | `timeLeft` | 剩餘時間 |
| `/ly/me/hp` | `myselfHealth`（取sentry值）、`friendRobots` | 我方血量 |
| `/ly/enemy/hp` | `enemyRobots` | 敵方血量 |
| `/ly/me/ammo_left` | `ammoLeft` | 子彈數 |
| `/ly/enemy/op_hp` | `enemyOutpostHealth` | 敵方前哨血量 |
| `/ly/me/op_hp` | `selfOutpostHealth` | 我方前哨血量 |
| `/ly/me/base_hp` | `selfBaseHealth` | 我方基地血量 |
| `/ly/team/buff` | `teamBuff` | 增益狀態 |
| `/ly/position/data` | `friendRobots`, `enemyRobots`（更新position） | UWB位置 |
| `/ly/gimbal/angles` | `gimbalAngles` | 當前雲台角 |
| `/ly/gimbal/posture` | `postureState` | 姿態回讀（0未知/1進攻/2防禦/3移動） |
| `/ly/gimbal/vel` | `naviVelocity` | 底盤速度反饋 |
| `/ly/predictor/target` | `autoAimData`, `isFindTargetAtomic` | 普通瞄準角度 |
| `/ly/outpost/target` | `outpostAimData`, `isFindTargetAtomic` | 前哨瞄準角度 |
| `ly/buff/target` | `buffAimData`, `isFindTargetAtomic` | 打符瞄準角度 |
| `/ly/gimbal/capV` | `capV` | 電容電壓 |

### `src/PublishMessage.cpp` — 所有發布函數

| 發布 Topic | 說明 |
|-----------|------|
| `/ly/control/angles` | 目標雲台角（`gimbalControlData.GimbalAngles`） |
| `/ly/control/firecode` | 火控碼（開火狀態、旋轉速度、瞄準模式） |
| `/ly/control/vel` | 底盤速度指令（導航） |
| `/ly/control/posture` | 姿態指令（0不下發/1進攻/2防禦/3移動） |
| `/ly/aa/enable` | 普通瞄準開關 |
| `/ly/ra/enable` | 打符模式開關 |
| `/ly/outpost/enable` | 前哨站瞄準開關 |
| `/ly/bt/target` | 當前打擊目標類型（→ `detector` 和 `predictor`） |
| `/ly/navi/goal` | 導航目標點位 |
| `/ly/navi/goal_pos` | 導航目標座標 |
| `/ly/navi/speed_level` | 底盤速度等級 |

### 比賽 Profile 切換

- `competition_profile:=league`
  - 使用聯盟賽簡化決策 profile
  - 若未顯式指定 `bt_config_file`，默認讀 `Scripts/ConfigJson/league_competition.json`
  - 建議搭配 `NaviSetting.UseXY=false`，只發 `/ly/navi/goal`
  - 聯盟賽會進入顯式策略 `LeagueSimple`，不再借用 `HitHero`
- `competition_profile:=regional`
  - 保持分區賽/原有複雜策略
  - 若未顯式指定 `bt_config_file`，默認回退到 `Scripts/config.json`
- `bt_config_file:=Scripts/ConfigJson/regional_competition.json`
  - 顯式指定某份 BT JSON 配置

推薦啟動方式（腳本入口）：

- `./scripts/start_sentry_all.sh`
  - 啟動時可選 `1/2`：`1=league`、`2=regional`
  - 腳本會自動對齊 `competition_profile` 與 `bt_config_file`
  - 腳本會強制 `config_file` 為實機配置 `scripts/config/auto_aim_config_competition.yaml`
- `./scripts/start_sentry_all.sh --mode 1 --no-prompt`
  - 非交互固定聯盟賽
- `./scripts/start_sentry_all.sh --mode 2 --no-prompt`
  - 非交互固定分區賽

### 聯盟賽決策（當前實作）

聯盟賽現在是單獨的一條簡化策略分支：

- 策略名：`LeagueSimple`
- 導航輸出：優先用 `/ly/navi/goal`
- 默認目標 ID：`OccupyArea = 18`
- 決策規則只有兩段：
  - 血量低 / 彈藥低：切 `Recovery`
  - 否則：切 `OccupyArea`

補充：

- `OccupyArea` 在 BT 只是目標序號，具體區域隨機點位應由導航側解釋。
- `Area::OccupyArea` 只保留兼容坐標，占位給舊的 `goal_pos` 鏈路，不建議聯盟賽依賴它。
- `bt_tree_file:=Scripts/main.xml`
  - 顯式指定 BT XML 文件

### 黑板結構（兩種賽制一致）

- 分區賽與聯盟賽都共用同一套 BT 主循環與雙黑板結構。
- `GlobalBlackboard_`：跨 tick 保留比賽狀態、策略模式、導航目標等。
- `TickBlackboard_`：每 tick 暫存中間決策資料。
- 差異只在策略分派與導航目標選擇，不在黑板架構。

---

### `module/BasicTypes.hpp` — 遊戲數據類型

| 類型 | 說明 |
|------|------|
| `AimMode` | 枚舉：AutoAim/Buff/Outpost/RotateScan |
| `ArmorType` | 枚舉：Hero=1, Engineer=2, Infantry1=3, Infantry2=4, Sentry=5, Outpost=7 |
| `UnitType` | 同上，但用於 Robot 對象索引 |
| `UnitTeam` | Red/Blue |
| `GimbalAnglesType` | {Yaw, Pitch}（float） |
| `GimbalControlData` | {GimbalAngles, FireCode, Velocity} |
| `FireCodeType` | 位域：FireStatus, Rotate(2bit), AimMode |
| `AimData` | {Angles(YawPitch), FireStatus, BuffFollow} |
| `RateClock` | 固定頻率時鐘（用毫秒計時） |
| `TimerClock` | 計時器（用於判斷是否到達某時刻） |
| `DescentDetector<T>` | 下降檢測器，用於血量減少判斷 |

### `module/Area.hpp` — 地圖區域定義

定義了比賽地圖上所有點位（`BuffShoot`、`OutpostShoot`、`MidShoot`、`CastleLeft` 等），用 `(x, y)` 座標表示，支持紅/藍兩隊鏡像：

```cpp
SET_POSITION(BuffShoot, MyTeam);  // 設置導航目標為打符點位
```

`Area::BuffShoot.near(x, y, 100, MyTeam)` 判斷當前位置是否在某點位附近100cm內。

---

## 訂閱/發布 Topic 完整匯總

### 訂閱（共15+個）
來自 `gimbal_driver`（遊戲狀態、血量、雲台角、子彈速度等）  
來自 `predictor`（`/ly/predictor/target`）  
來自 `outpost_hitter`（`/ly/outpost/target`）  
來自 `buff_hitter`（`ly/buff/target`）

### 發布（共10+個）
控制類：`/ly/control/angles`, `/ly/control/firecode`, `/ly/control/vel`, `/ly/control/posture`  
模式切換：`/ly/aa/enable`, `/ly/ra/enable`, `/ly/outpost/enable`  
目標廣播：`/ly/bt/target`（→ `detector`, `predictor`）  
導航：`/ly/navi/*`

---

## 黑板与后续方向

当前实现已使用 BT v4 主树执行，且在运行期维护全局黑板与 tick 黑板。  
后续若继续拆分，可沿「感知黑板 / 决策黑板」分层，以降低 `Application` 聚合状态复杂度。
