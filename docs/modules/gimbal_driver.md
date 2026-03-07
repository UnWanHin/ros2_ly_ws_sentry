# gimbal_driver — 雲台驅動節點

## 概述

`gimbal_driver` 是整個系統的**硬件接口層**，負責與底層電控板（stm32/串口）雙向通信：
- **讀取方向**：從串口讀取電控數據（雲台角度、比賽狀態、血量、子彈速度等），解析後發布成多個 ROS2 Topic
- **寫入方向**：訂閱決策節點（`behavior_tree`）發來的控制指令，轉發給電控板

**整個系統的唯一硬件接入點**，所有感知和決策靠這個節點提供狀態。

---

## 目錄結構

```
gimbal_driver/
├── CMakeLists.txt
├── package.xml
├── main.cpp                    # 主節點，Application 類
├── launch/
│   ├── gimbal_driver.launch.py  # ROS 2 主入口（推薦）
│   └── gimbal_driver.launch     # ROS 2 XML 兼容入口
├── msg/                        # 自定義消息類型
│   ├── GimbalAngles.msg        # 雲台角度
│   ├── GameData.msg            # 比賽數據（彙總）
│   ├── Health.msg              # 各機器人血量
│   ├── BuffData.msg            # 能量機關增益狀態
│   ├── PositionData.msg        # 機器人位置（UWB）
│   ├── UWBPos.msg              # UWB位置
│   └── Vel.msg                 # 速度指令
└── module/
    ├── BasicTypes.hpp          # 底層數據結構定義
    ├── IODevice.hpp            # 串口讀寫封裝
    ├── ROSTools.hpp            # ROS工具（ROSNode、LY_DEF_ROS_TOPIC等）
    ├── crc_checker.hpp         # CRC校驗
    ├── no_ranges_error.hpp     # 範圍錯誤處理
    └── pp_span.hpp             # 數據包解析輔助
```

---

## 單獨啟動（建議）

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py
```

虛擬串口測試：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py use_virtual_device:=true
```

兼容 XML 入口：

```bash
ros2 launch gimbal_driver gimbal_driver.launch
```

---

## 核心文件詳解

### `main.cpp` — 主節點（Application 類）

整個節點是一個 `Application` 類，包含一個 `ROSNode` 和一個 `IODevice`（串口設備）。

**啟動流程**：
```
main()
  └── app.Run()
        ├── Node.Initialize(argc, argv)   // 初始化 ROS 節點
        ├── GenSubs()                     // 生成訂閱者（接收控制指令）
        └── 主循環:
              ├── Device.Initialize()     // 嘗試打開串口
              ├── std::jthread: LoopRead()  // 後台持續讀串口
              └── while(!DeviceError): rclcpp::spin_some()  // 處理回調，發送控制數據
```

#### 「讀取」路徑：`LoopRead()` → `Pub*()`

從串口讀到 `TypedMessage`，根據 `TypeID` 分發到不同的 `Pub*` 函數：

| TypeID 對應數據結構 | 調用函數 | 發布的 Topic |
|---|---|---|
| `GimbalData` | `PubGimbalData()` | `/ly/gimbal/angles`, `/ly/gimbal/firecode`, `/ly/gimbal/vel`, `/ly/gimbal/capV` |
| `GameData` | `PubGameData()` | `/ly/game/all`, `/ly/me/ammo_left`, `/ly/enemy/op_hp`, `/ly/me/is_team_red`, `/ly/game/is_start`, `/ly/game/time_left`, 等 |
| `HealthMyselfData` | `PubHealthMyselfData()` | `/ly/me/hp`, `/ly/me/base_hp` |
| `HealthEnemyData` | `PubHealthEnemyData()` | `/ly/enemy/hp`, `/ly/enemy/base_hp` |
| `RFIDAndBuffData` | `PubRFIDAndBuffData()` | `/ly/me/rfid`, `/ly/team/buff` |
| `PositionData` | `PubPositionData()` | `/ly/position/data`, `/ly/me/uwb_pos`, `/ly/bullet/speed` |
| `ExtendData` | `PubExtendData()` | `/ly/me/uwb_yaw`, `/ly/gimbal/posture` |

#### 「寫入」路徑：`GenSubs()` → `Device.Write()/WriteRaw()`

訂閱上層控制指令，寫入共享的 `GimbalControlData` 結構體，然後由 `CallbackGenerator` 觸發寫入串口：

| 訂閱 Topic | 對應字段 | 說明 |
|---|---|---|
| `/ly/control/angles` (`GimbalAngles`) | `GimbalControlData.GimbalAngles.Yaw/Pitch` | 期望雲台角 |
| `/ly/control/firecode` (`UInt8`) | `GimbalControlData.FireCode` | 開火指令、旋轉模式 |
| `/ly/control/vel` (`Vel`) | `GimbalControlData.Velocity.X/Y` | 底盤速度（導航用） |
| `/ly/control/posture` (`UInt8`) | `postureCommand_` + 獨立姿態幀 | 姿態指令（0=不控制, 1=進攻, 2=防禦, 3=移動） |

姿態下發採用「獨立下行幀」策略：
- 不改既有 `GimbalControlData` 包長（舊下位機兼容）
- 收到有效姿態命令（1/2/3）後由 `WriteRaw()` 下發 `PostureControlMessage`（`TypeID=7`，payload 1 byte posture）
- 失聯重連後會自動重發當前姿態

---

### `module/BasicTypes.hpp` — 底層數據結構

定義了所有與電控板通信的二進制數據結構（用 `#pragma pack`/位域）：

| 結構體 | 說明 |
|--------|------|
| `GimbalData` | 電控→上位機：雲台角、速度、開火狀態等 |
| `GimbalControlData` | 上位機→電控：期望雲台角、開火指令、速度 |
| `PostureControlMessage` | 上位機→電控：姿態獨立命令幀（`TypeID=7`，`Data[0]=posture`） |
| `GameData` | 裁判系統數據：比賽狀態、血量、子彈數、時間 |
| `HealthMyselfData` / `HealthEnemyData` | 我方/敵方各機器人血量 |
| `RFIDAndBuffData` | RFID狀態 + 能量機關增益（防禦、攻擊、回血等） |
| `PositionData` | UWB定位數據（友/敵機器人X、Y座標）+ 子彈速度 |
| `ExtendData` | UWB yaw 角（自身朝向） |
| `FireCodeType` | 位域：`FireStatus`（開火狀態）、`Rotate`（旋轉速度0-3）、`AimMode`（瞄準模式） |

---

### `module/IODevice.hpp` — 串口設備封裝

```cpp
template<typename ReadType, typename WriteType>
class IODevice {
    bool Initialize(bool useVirtualDevice = false);  // 打開串口（或虛擬迴環）
    bool Write(const WriteType& data);               // 寫入電控
    bool WriteRaw(const OtherType& data);            // 寫入任意二進制幀（姿態獨立幀使用）
    void LoopRead(std::atomic_bool& error, std::function<void(const ReadType&)> callback);  // 持續讀取
};
```

**虛擬設備模式**（`useVirtualDevice=true`）：用於在沒有硬件時做本地迴環測試，通過 `TestVirtualLoopback()` 驗證數據收發。

---

### `module/ROSTools.hpp` — ROS工具宏

`gimbal_driver/module/ROSTools.hpp` 和 `auto_aim_common/include/RosTools/RosTools.hpp` 功能類似，但 `gimbal_driver` 自帶了一份本地版本用於自身的 `MultiCallback` 機制。

**`MultiCallback<GimbalControlData>`**：線程安全的多訂閱者回調彙總器。多個訂閱者回調各自更新 `GimbalControlData` 的不同字段，最後由回調函數觸發一次 `Device.Write()`。

---

## msg/ 消息格式詳解

### `GimbalAngles.msg`

**Topic**：`/ly/gimbal/angles`（讀取）、`/ly/control/angles`（寫入）

| 字段 | 類型 | 說明 |
|------|------|------|
| `yaw` | `float32` | 當前 yaw 角（度，絕對角） |
| `pitch` | `float32` | 當前 pitch 角（度，向下為正） |

### `GameData.msg`

**Topic**：`/ly/game/all`

| 字段 | 類型 | 說明 |
|------|------|------|
| `gamecode` | `uint16` | 裁判系統遊戲狀態碼（包含比賽開始、哨兵血量、隊伍顏色、英雄警告等位域） |
| `ammoleft` | `uint16` | 剩餘子彈數 |
| `timeleft` | `uint16` | 比賽剩餘時間（秒） |
| `selfhealth` | `uint16` | 自身血量 |
| `exteventdata` | `uint32` | 擴展事件數據（RFID、補給站等） |

### `Health.msg`

**Topic**：`/ly/me/hp`, `/ly/enemy/hp`

| 字段 | 說明 |
|------|------|
| `hero`, `engineer`, `infantry1`, `infantry2`, `reserve`, `sentry` | 各機器人血量（uint16） |

### `BuffData.msg`

**Topic**：`/ly/team/buff`

| 字段 | 說明 |
|------|------|
| `recoverybuff`, `coolingbuff`, `defencebuff`, `vulnerabilitybuff`, `attackbuff` | 各增益激活標誌（uint8） |
| `remainingenergy` | 剩餘底盤能量（5位，0b10000=5%） |

### `PositionData.msg`

**Topic**：`/ly/position/data`

| 字段 | 說明 |
|------|------|
| `friendcarid`, `friendx`, `friendy` | 友方車輛ID和UWB座標（cm） |
| `enemycarid`, `enemyx`, `enemyy` | 敵方車輛ID和UWB座標（cm） |

---

## Topic 匯總

### 發布的 Topics

| Topic | 消息類型 | 說明 |
|-------|----------|------|
| `/ly/gimbal/angles` | `GimbalAngles` | **最重要**：雲台當前角度，`detector` 和 `behavior_tree` 都需要 |
| `/ly/me/is_team_red` | `Bool` | 我方是否紅隊 |
| `/ly/game/is_start` | `Bool` | 比賽是否開始 |
| `/ly/game/time_left` | `UInt16` | 剩餘時間 |
| `/ly/me/hp` | `Health` | 我方各機器人血量 |
| `/ly/enemy/hp` | `Health` | 敵方各機器人血量 |
| `/ly/me/ammo_left` | `UInt16` | 剩餘子彈 |
| `/ly/bullet/speed` | `Float32` | 子彈速度（m/s，從 PositionData 解析） |
| `/ly/team/buff` | `BuffData` | 能量機關增益狀態 |
| `/ly/position/data` | `PositionData` | UWB位置數據 |
| `/ly/me/uwb_pos` | `UInt16MultiArray` | 自身UWB位置[x, y] |
| `/ly/gimbal/posture` | `UInt8` | 姿態回讀（約定來源為 `ExtendData.Reserve_16` 低 2 bit，僅 1/2/3 視為有效） |

### 訂閱的 Topics

| Topic | 消息類型 | 說明 |
|-------|----------|------|
| `/ly/control/angles` | `GimbalAngles` | 接收決策節點的目標角度 |
| `/ly/control/firecode` | `UInt8` | 接收開火指令 |
| `/ly/control/vel` | `Vel` | 接收速度指令（導航） |
| `/ly/control/posture` | `UInt8` | 接收姿態指令（上位決策輸入） |

### 姿態下發固定协議

| 字段 | 固定值 | 说明 |
|---|---:|---|
| `TypeID` | `7` | 姿态独立幀 ID（需下位机按此解析） |
| `repeat_count` | `3` | 每次切换默认重发 3 次 |
| `repeat_interval_ms` | `20` | 重发间隔 20ms |

---

## 修改注意事項

- **串口協議調整**：修改 `module/BasicTypes.hpp` 的結構體時一定要注意字節對齊和電控端的協議版本一致
- **姿態指令兼容策略**：姿態不併入 `GimbalControlData`；改用固定 `TypeID=7` 獨立幀，對舊主包零影響
- **新增 Topic**：在 `main.cpp` 增加 `LY_DEF_ROS_TOPIC` 定義和對應的 `Pub*()` 函數，並在 `LoopRead()` 的 switch-case 中處理
- **`/ly/bullet/speed`**：子彈速度從 `PositionData` 解析，換算公式為 `data.BulletSpeed / 100.0f`（原始值為 cm/s × 100 的整數）
- **虛擬設備**：調試時可設置 YAML 參數 `io_config/use_virtual_device: true` 來使用迴環模式而無需電控硬件
