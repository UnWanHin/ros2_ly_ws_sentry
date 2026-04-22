# predictor — 預測與控制節點

## 概述

`predictor` 是自動瞄準鏈路中的最後計算環節，接收 `tracker_solver` 輸出的 **IMU 座標系下的跟蹤結果**，計算預測彈道，輸出雲台的最終期望角度。

**功能**：
1. **預測（Predictor）**：根據目標的歷史運動（EKF擴展卡爾曼）預測未來位置（補前置量）
2. **控制（Controller）**：根據目標狀態、當前雲台角、彈速，計算最優瞄準的 yaw/pitch
3. **彈道解算（Solver）**：計算拋物線彈道補償（重力補償）

---

## 目錄結構

```
predictor/
├── CMakeLists.txt
├── package.xml
├── predictor_node.cpp          # 主節點（PredictorNode 類）
├── config/
│   └── predictor_config.yaml
├── include/
│   ├── predictor/
│   │   └── predictor.hpp       # 預測器接口
│   ├── controller/
│   │   └── controller.hpp      # 控制器接口
│   └── solver/
│       └── solver.hpp          # 彈道解算接口
└── src/
    ├── predictor.cpp           # EKF預測器實現
    ├── motion_model.cpp        # 運動模型（勻速/圓周/陀螺等）
    ├── controller.cpp          # 目標選擇、角度計算
    └── solver.cpp              # 彈道解算（重力補償）
```

---

## 核心文件詳解

### `predictor_node.cpp` — 主節點（PredictorNode 類）

#### 初始化順序（同樣的兩段設計）

```cpp
int main() {
    rclcpp::init();
    auto app = make_shared<PredictorNode>();  // 構造：只初始化 ROSNode
    
    // 設置全局指針（三個！）
    ly_auto_aim::controller::global_controller_node = node_ptr;
    ly_auto_aim::solver::global_predictor_solver_node = node_ptr;
    ly_auto_aim::predictor::global_predictor_node = node_ptr;
    
    app->Init();  // 此時才 createSolver/createPredictor/createController
    rclcpp::spin(node_ptr);
}
```

`Init()` 中：
1. `createSolver()`：創建彈道解算器（讀相機內參、重力參數）
2. `createPredictor()`：創建EKF預測器
3. `createController()`：創建控制器
4. `Location::registerSolver(solver)`：注冊到全局Location管理器
5. 設置回調：`controller->registPredictFunc()`，讓 controller 可以調用 predictor 的預測函數
6. 生成訂閱者

#### 訂閱的 Topics

| Topic | 回調函數 | 說明 |
|-------|----------|------|
| `/ly/tracker/results` | `predictor_callback` | 核心回調，接收跟蹤結果 |
| `/ly/bt/target` | `get_target_callback` | 接收 behavior_tree 設定的打擊目標 |
| `/ly/bullet/speed` | `get_bullet_speed_callback` | 接收實時彈速（用於彈道計算） |

#### 發布的 Topics

| Topic | 消息類型 | 說明 |
|-------|----------|------|
| `/ly/predictor/target` | `Target` | 最終瞄準角度（固定頻率計算，但僅在 `status=true` 且 `yaw/pitch` 有限值時才發給 `behavior_tree`） |
| `/ly/predictor/debug` | `DebugFilter` | 調試信息（僅在 `status=true` 且有 prediction 時發布） |

#### 当前结构：订阅 update + timer publish

当前 `predictor_node` 已拆成两段：

1. `predictor_callback()`
   - 接收 `/ly/tracker/results`
   - 只更新内部模型与最近观测
2. `publish_timer_callback()`
   - 固定 `10ms`
   - 调 `controller->control(...)`
   - 发布 `/ly/predictor/target`

#### `predictor_callback()` — 观测更新流程

```
predictor_callback(Trackers.msg)
│
├── GimbalAngleType gimbal_angle{msg->pitch, msg->yaw}
│
├── if(!Location::isSolverRegistered()) return  // 安全檢查，防止未初始化就處理
│
├── convertMsgToTrackResults()    // Trackers.msg → TrackResultPairs（含IMU座標）
│
├── timestamp = rclcpp::Time(msg->header.stamp).seconds()  // 清洗時間戳
├── predictor->update(track_results, timestamp)  // 用新觀測更新EKF
├── 記錄最近一次 gimbal_angle / header / observation time
└── 返回，等待 timer 發布
```

#### `publish_timer_callback()` — 固定频率计算、条件式发布流程

```
publish_timer_callback()  // 10ms
│
├── predictions = predictor->predict(now)
├── has_predictions ? observation_fresh ?
│
├── if 沒 prediction 且觀測已 stale:
│   └── 不對 BT 發 `/ly/predictor/target`
│
└── else:
    ├── control_result = controller->control(...)
    ├── status = control_result.valid
    ├── yaw = control_result.yaw_actual_want
    ├── pitch = control_result.pitch_actual_want
    └── 仅在 `status=true` 且角度有限时 publish /ly/predictor/target
```

---

### `src/predictor.cpp` + `include/predictor/predictor.hpp`

**`Predictor`** — 擴展卡爾曼濾波器（EKF）

核心功能：
- `update(track_results, timestamp)`：用新觀測更新濾波器狀態
- `predict(timestamp)`：預測指定時刻的目標位置，返回 `vector<Prediction>`

**`Prediction`** 包含：
- `center`（XYZ）：預測的車中心座標
- `theta`：當前旋轉角（車體yaw）
- `omega`：角速度（yaw轉速，關鍵！決定裝甲板在哪裡）
- `vx`, `vy`：水平速度
- `r1`, `r2`：裝甲板到車中心的旋轉半徑（前後兩塊不同）
- `z2`：後裝甲板的高度

---

### `src/motion_model.cpp`

**`MotionModel`** — EKF 的運動模型庫

| 模型 | 說明 |
|------|------|
| 勻速直線模型 | 適合直線移動的目標 |
| 圓周運動模型 | 適合旋轉的目標（小陀螺） |
| 裝甲板旋轉模型 | 考慮車體旋轉，預測哪一塊裝甲板可打 |

`predictor.cpp` 根據目標的運動特徵自動選擇或切換模型。

---

### `src/controller.cpp` + `include/controller/controller.hpp`

**`Controller`** — 目標選擇 + 角度計算

`control(gimbal_angle, target_id, bullet_speed)` 的流程：
1. 從所有跟蹤目標中，找到 `car_id == target_id` 的目標
2. 調用 `predictor->predict()` 獲取預測位置
3. 計算哪一塊裝甲板當前可打（根據 `theta` 和 `omega` 計算各裝甲板的朝向角）
4. 選出最容易打的裝甲板（`armor_id` 選擇）
5. 加上提前量（根據飛行時間 × 速度）
6. 輸出 `ControlResult.yaw_actual_want` 和 `.pitch_actual_want`

`registPredictFunc()` 是為了讓 controller 能在計算瞄準角時調用 predictor 的預測（迭代求解飛行時間vs提前量）。

---

### `src/solver.cpp` + `include/solver/solver.hpp`

**`Solver`** — 彈道解算器

把3D坐標轉換為雲台角，並加入重力補償：

```
xyz_imu (3D目標位置) 
    → 水平距離 + 垂直距離
    → 解拋物線方程 (需要 bullet_speed, 重力加速度)
    → pitch 補償角
    → 最終 yaw, pitch
```

**全局指針**：
```cpp
extern shared_ptr<rclcpp::Node> global_predictor_solver_node;
```
`createSolver()` 通過此指針讀取相機內參和彈道參數。

---

### `ControlResult` 結構

```cpp
struct ControlResult {
    bool valid;             // 是否有有效目標
    float yaw_actual_want;  // 期望雲台 yaw（世界座標，度）
    float pitch_actual_want;// 期望雲台 pitch（度）
};
```

当前 `predictor_node` 不再向 `behavior_tree` 持续发布无效 target：

- `valid=true`：发布 `/ly/predictor/target`
- `valid=false`：只保留内部诊断日志，不再把 `status=false + NaN` 喂给 `behavior_tree`

---

## 全局原子量

| 變量 | 說明 |
|------|------|
| `automic_target` | `ArmorType` 枚舉，當前要打的目標類型（由 `/ly/bt/target` 更新） |
| `atomic_bullet_speed` | `float`，默認 23.0m/s（由 `/ly/bullet/speed` 更新） |

---

## 數據流

```
/ly/tracker/results (Trackers.msg)
         │
         ▼
PredictorNode::predictor_callback()
├── predictor->update()       ← 用觀測更新EKF
└── 緩存最近觀測
         │
         ▼
PredictorNode::publish_timer_callback()  // 10ms
├── predictor->predict()
├── controller->control()
├── publish(valid only) → /ly/predictor/target
└── publish → /ly/predictor/debug (條件式)
```

---

## 修改注意事項

- **彈速**：`atomic_bullet_speed` 默認 23.0f，收到 `/ly/bullet/speed` 后会更新；`controller.cpp` 当前会对弹速做下限与平滑处理，不再强制回写固定 `23.0f`
- **時間戳清洗**：`predictor->update()` 和 `predict()` 的時間戳都必須先轉 double 秒，否則 ROS2 不同時鐘源會異常
- **`valid` 判斷**：当前 `status=false` 的主要原因可以通过 `predictor_node` 的节流日志看到，如 `no_predictions_and_stale`、`no_prediction` 等；但这些无效结果不再发给 `behavior_tree`
- **EKF發散**：當 `v_yaw`（角速度）估計發散時（小陀螺高速旋轉），可能出現瞄準角跳動，需要在 `motion_model.cpp` 調整過程噪聲 Q 矩陣
