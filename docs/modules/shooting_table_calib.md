# shooting_table_calib — 彈道標定節點

## 概述

`shooting_table_calib`（射擊表標定）是一個**獨立的標定工具節點**，用於在比賽前實際測量各種距離下子彈的彈道誤差，生成二維多項式補償係數，供 `predictor/solver` 使用。

**功能**：直接融合了 `detector`、`tracker_solver`、`predictor` 三個節點的所有功能，形成一個閉環：**看到靶板 → 自動瞄準 → 記錄落點誤差 → 擬合補償曲線**。

---

## 目錄結構

```
shooting_table_calib/
├── CMakeLists.txt / package.xml
├── README.md                       # 標定使用說明（3KB）
├── shooting_table_node.cpp         # 主節點（全部邏輯，957行）
├── launch/
│   ├── shooting_table_calib.launch.py  # ROS 2 主入口（推薦）
│   └── shooting_table_calib.launch     # ROS 2 XML 兼容入口
└── include/                        # 空目錄
```

---

## 核心文件：`shooting_table_node.cpp`（957行）

### `ShootingTableCalibNode` 類

這個節點**直接包含**了 detector、tracker_solver、predictor 的核心算法對象（不走 ROS Topic）：

| 成員對象 | 類型 | 來源包 |
|----------|------|--------|
| `carAndArmorDetector` | `CarAndArmorDetector` | `detector` |
| `filter` | `ArmorFilter` | `detector` |
| `finder` | `ArmorRefinder` | `detector` |
| `camera` | `Camera` | `detector` |
| `tracker` | `tracker::Tracker` | `tracker_solver` |
| `solver` | `solver::Solver` | `tracker_solver` |
| `predictor` | `predictor::Predictor` | `predictor` |
| `controller` | `controller::Controller` | `predictor` |

**初始化順序**（構造函數）：
```
loadShootTableParams()   // 從ROS參數讀取射表係數
initializeVideo()        // 配置視頻/相機模式
initializeCamera()       // 初始化大恆相機
initializeDetector()     // 加載YOLO+分類器模型
initializeAlgorithms()   // 創建tracker/solver/predictor/controller
setupRosTopics()         // 設置訂閱/發布
createCSVFile()          // 創建 CSV 輸出文件
```

---

### 鍵盤交互界面

節點通過 `KeyboardInput` 類實現**無阻塞鍵盤輸入**（`termios` 非規範模式）：

| 按鍵 | 功能 |
|------|------|
| `a` | 鎖定目標（自動瞄準並啟動控制） |
| `s` | 保存當前標定記錄到 CSV |
| `k` | 停止瞄準和射擊 |
| `o` | 切換「僅瞄準」模式（只發雲台角，不發火控） |
| `↑`/`↓` | pitch 微調（±0.1°/次） |
| `←`/`→` | yaw 微調（±0.1°/次） |
| `p` | 輸入鍵修改射擊表係數（交互輸入模式） |
| `r` | 顯示當前參數 |

---

### `processImageDetections()` — 核心感知循環

```
獲取圖像（相機/視頻）
    ↓
carAndArmorDetector.Detect()    // YOLO 檢測
    ↓
filter.Filter()                  // 顏色+類型過濾（默認只打 Infantry2）
    ↓
tracker->merge()                 // 跟蹤更新
    ↓
solver->solve_all()              // 圖像座標→IMU座標
    ↓
if(should_aim_once):
    calculateBallisticSolution() // 彈道解算（迭代牛頓法求拋物線）
    → target_pitch, target_yaw
    → 鎖定目標，is_aiming=true
```

---

### `calculateBallisticSolution()` — 彈道解算（牛頓迭代）

```
輸入：target_xyz（IMU座標系下3D位置，米）
計算：
  GRAVITY = 9.794 m/s²
  bullet_speed = 23.0 m/s（硬編碼）
  迭代100次牛頓法求 theta（仰角）使子彈落在目標高度
輸出：
  target_pitch = theta（°）
  target_yaw = atan2(y, x)（°，轉換為絕對角對齊雲台）
```

---

### 射擊表格式（`ShootingRecord`）

每條記錄寫入 CSV 的字段：

| 字段 | 說明 |
|------|------|
| `timestamp` | 記錄時刻（秒） |
| `z_height` | 目標Z高度（m） |
| `horizontal_distance` | 水平距離（m） |
| `relative_yaw/pitch` | 手動微調量（鍵盤偏移） |
| `target_x/y/z` | 目標世界座標 |
| `absolute_yaw/pitch` | 記錄時的絕對雲台角 |
| `target_yaw` | 解算的目標yaw |
| `fitted_pitch/yaw` | 用二維多項式係數擬合的預測補償值 |

---

### 二維多項式射擊表模型

補償量 = f(z_height, horizontal_distance)：

```
pitch_compensation = intercept 
    + coef_z × z 
    + coef_d × d 
    + coef_z2 × z² 
    + coef_zd × z×d 
    + coef_d2 × d²
```

係數通過 ROS2 參數服務器讀取（可在 `launch/*.py` 中配置），每3秒自動重新讀取一次（支持運行時參數調整）。

---

### Topic 連接

| 方向 | Topic | 說明 |
|------|-------|------|
| 訂閱 | `/ly/gimbal/angles` | 讀取當前雲台角（用於 yaw 對齊計算） |
| 發布 | `/ly/control/angles` | 直接發送雲台目標角（繞過 behavior_tree） |
| 發布 | `/ly/predictor/target` | 發送 Target.msg（status=true 表示瞄準有效） |
| 發布 | `/ly/control/firecode` | 開火控制（`0b00`=停/`0b11`=開火） |

---

## ⚠️ 修改注意事項

- **彈速硬編碼**：`bullet_speed = 23.0` 寫死，應從 `/ly/bullet/speed` 動態獲取
- **CSV 路徑硬編碼**：`std::getenv("HOME") + "/workspace/record"`，需要與你實際的 `record/` 目錄對齊（當前在 `~/ros2_ly_ws_sentary/src/record/`）
- **目標類型固定**：`target = ArmorType::Infantry2`，只打步兵2號，標定時確認靶板類型
- **`should_aim_once` 機制**：原子量，按 `a` 鍵觸發一次瞄準，避免主循環持續更新目標打亂手動微調

---

## 標定流程（参考 README.md）

1. 啟動 `gimbal_driver` + `shooting_table_calib`
   `ros2 launch shooting_table_calib shooting_table_calib.launch.py`
2. 將靶板放在不同距離（建議 2m/3m/4m/5m）
3. 每個距離按 `a` 鎖定靶板
4. 用 `↑↓←→` 微調校正偏差
5. 按 `s` 保存一條記錄
6. 重複3-5直到數據足夠（各距離至少5-10條）
7. 用外部工具（Python/Matlab）做二次多項式擬合，填入 launch 文件的參數
