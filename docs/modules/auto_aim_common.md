# auto_aim_common — 公共接口包

## 概述

`auto_aim_common` 是整個 `ros2_ly_ws_sentary` 工作空間的**核心公共依賴包**，不包含任何可執行節點。它只定義了兩類東西：
1. **ROS2 消息類型（msg/）** — 各節點之間通信的消息格式
2. **C++ 公共頭文件（include/auto_aim_common/）** — 被其他所有包 `#include` 的工具類和類型定義

所有其他包（`detector`、`tracker_solver`、`predictor`、`behavior_tree` 等）都 `depend` 於此包。

---

## 目錄結構

```
auto_aim_common/
├── CMakeLists.txt          # 構建定義，使用 rosidl 生成消息
├── package.xml             # ROS2 包描述，依賴 rclcpp, std_msgs, geometry_msgs, sensor_msgs
├── msg/                    # ROS2 消息定義
│   ├── Armor.msg
│   ├── Armors.msg
│   ├── ArmorTracker.msg
│   ├── Car.msg
│   ├── CarTracker.msg
│   ├── Trackers.msg
│   ├── Target.msg
│   ├── AngleImage.msg
│   ├── DebugFilter.msg
│   └── Rect.msg
└── include/
    └── auto_aim_common/    # 公共 C++ 頭文件
        ├── DetectionType.hpp
        ├── TrackerType.hpp
        ├── PredictionType.hpp
        ├── Location.hpp
        └── ...
```

---

## msg/ — 消息格式詳解

### `Armor.msg` — 單個裝甲板

**發布者**：`detector`  
**訂閱者**：作為 `Armors.msg` 的子消息使用

| 字段 | 類型 | 說明 |
|------|------|------|
| `type` | `int32` | 裝甲板數字類型（英雄=1, 工程=2, 步兵=3/4, 哨兵=5, 前哨=7） |
| `color` | `int32` | 顏色：0=藍, 1=紅, 2=灰, 3=其他 |
| `distance` | `float32` | 到裝甲板的距離（米） |
| `distance_to_image_center` | `float32` | 裝甲重心到圖像中心的像素距離，用於 `ArmorFilter` 選優先目標 |
| `corners_x[]` | `float32[]` | 裝甲板4個角點的圖像X座標（順序：左上、右上、右下、左下） |
| `corners_y[]` | `float32[]` | 裝甲板4個角點的圖像Y座標 |
| `rotation[]` | `float32[]` | PnP解算得到的旋轉向量（Rodrigues，3元素） |
| `translation[]` | `float32[]` | PnP解算得到的位移向量（相機座標系XYZ，3元素） |

---

### `Armors.msg` — 一幀的所有裝甲板

**Topic**：`/ly/detector/armors`（普通瞄準）、`/ly/outpost/armors`（前哨站瞄準）  
**發布者**：`detector`  
**訂閱者**：`tracker_solver`, `outpost_hitter`

| 字段 | 類型 | 說明 |
|------|------|------|
| `header` | `std_msgs/Header` | 時間戳（綁定該幀的拍攝時刻）、frame_id="camera" |
| `armors[]` | `Armor[]` | 本幀所有已過濾和PnP解算的裝甲板列表 |
| `cars[]` | `Car[]` | 本幀所有YOLO檢測到的車輛列表 |
| `yaw` | `float32` | 拍攝這一幀時的**雲台yaw角**（用於時間補償，解決通信延時問題） |
| `pitch` | `float32` | 拍攝這一幀時的**雲台pitch角** |
| `is_available_armor_for_predictor` | `bool` | 是否有可供預測器使用的裝甲板 |
| `target_armor_index_for_predictor` | `int16` | 推薦給預測器的裝甲板索引 |

> **設計意圖**：`yaw` 和 `pitch` 字段是為了解決「拍攝到發布之間存在延遲」的問題。把雲台數據與圖像時間戳綁定，讓下游節點知道這幀圖像拍攝時雲台在哪個方向。

---

### `Car.msg` — 車輛檢測結果

**嵌入在** `Armors.msg.cars[]` 中

| 字段 | 類型 | 說明 |
|------|------|------|
| `bounding_rect` | `Rect` | YOLO 檢測到的車輛邊界框 |
| `car_id` | `int32` | 車輛ID（同裝甲板 type 編號） |

---

### `ArmorTracker.msg` — 裝甲板跟蹤結果

**Topic**：作為 `Trackers.msg.armor_trackers[]` 的元素  
**發布者**：`tracker_solver`  
**訂閱者**：`predictor`

| 字段 | 類型 | 說明 |
|------|------|------|
| `x`, `y`, `z` | `float64` | 裝甲板在 **IMU 座標系**下的3D位置（米） |
| `yaw` | `float64` | 車體的 yaw 角（由裝甲板法向量推算） |
| `armor_id` | `int32` | 裝甲板編號（同一輛車有多個裝甲板，用此區分） |
| `car_id` | `int32` | 所屬車輛ID |

---

### `CarTracker.msg` — 車輛跟蹤結果

**嵌入在** `Trackers.msg.car_trackers[]` 中

| 字段 | 類型 | 說明 |
|------|------|------|
| `car_id` | `int32` | 車輛ID |
| `bounding_rect` | `Rect` | 跟蹤到的車輛邊界框 |

---

### `Trackers.msg` — 一幀的所有跟蹤結果

**Topic**：`/ly/tracker/results`  
**發布者**：`tracker_solver`  
**訂閱者**：`predictor`

| 字段 | 類型 | 說明 |
|------|------|------|
| `header` | `std_msgs/Header` | 繼承自 `Armors.msg` 的時間戳 |
| `armor_trackers[]` | `ArmorTracker[]` | 所有裝甲板的跟蹤結果（IMU座標系） |
| `car_trackers[]` | `CarTracker[]` | 所有車輛的跟蹤結果 |
| `yaw`, `pitch` | `float32` | 該幀的雲台角（繼承自上游 `Armors.msg`） |

---

### `Target.msg` — 最終瞄準指令

**Topic**：`/ly/predictor/target`（普通瞄準）、`/ly/outpost/target`（前哨站）、`/ly/buff/target`（打符）  
**發布者**：`predictor`、`outpost_hitter`、`buff_hitter`  
**訂閱者**：`behavior_tree`（彙總後控制雲台）

| 字段 | 類型 | 說明 |
|------|------|------|
| `header` | `std_msgs/Header` | 時間戳 |
| `status` | `bool` | `true` = 有效目標（可以開火），`false` = 無目標 |
| `buff_follow` | `bool` | 打符模式下是否跟蹤符（控制是否跟符轉動） |
| `yaw` | `float32` | 期望雲台 yaw 角（世界座標系，度） |
| `pitch` | `float32` | 期望雲台 pitch 角（度） |

---

### `AngleImage.msg` — 帶角度的圖像

**Topic**：`/ly/ra/angle_image`  
**發布者**：`detector`（將相機圖像 + 當時雲台角打包）  
**訂閱者**：`buff_hitter`

| 字段 | 類型 | 說明 |
|------|------|------|
| `image` | `sensor_msgs/Image` | 原始BGR圖像 |
| `yaw` | `float32` | 圖像拍攝時的雲台yaw |
| `pitch` | `float32` | 圖像拍攝時的雲台pitch |

> **設計意圖**：`buff_hitter`（打符模塊）需要**圖像 + 當時雲台角**才能計算符的位置，`AngleImage` 將兩者打包同步傳遞，避免時序問題。

---

### `DebugFilter.msg` — 預測器調試信息

**Topic**：`/ly/predictor/debug`  
**發布者**：`predictor`  
**訂閱者**：調試工具/可視化

| 字段 | 類型 | 說明 |
|------|------|------|
| `tracking` | `bool` | 是否正在跟蹤目標 |
| `position` | `geometry_msgs/Point` | 預測的車中心座標（xc, yc, zc） |
| `yaw` | `float32` | 預測的車體yaw角 |
| `velocity` | `geometry_msgs/Vector3` | 車的速度向量（vx, vy, vz） |
| `v_yaw` | `float32` | 車體yaw角速度 |
| `radius_1`, `radius_2` | `float32` | 當前與上一次預測的裝甲板旋轉半徑 |
| `z_2` | `float32` | 上一次預測的裝甲板z高度 |

---

## include/auto_aim_common/ — 公共頭文件

| 文件 | 作用 |
|------|------|
| `DetectionType.hpp` | 定義 `Detection`（裝甲板像素角點）、`CarDetection`（車輛BBox）、`Detections` 等基礎類型，供 `detector`→`tracker_solver` 轉換使用 |
| `TrackerType.hpp` | 定義 `TrackResult`（裝甲板跟蹤結果含IMU座標）、`CarTrackResult`、`TrackResultPairs`，供 `tracker_solver`→`predictor` 使用 |
| `PredictionType.hpp` | 定義 `Prediction`（預測結果，含中心、半徑、速度、角速度），供 `predictor` 內部使用 |
| `Location.hpp` | 定義 `location::Location` 類和靜態的 `registerSolver()` / `isSolverRegistered()`，提供全局 Solver 單例管理 |
| `Rect.msg` | 矩形框（x, y, width, height），用於車輛 BBox |

---

## 數據流關係圖

```
[gimbal_driver] ──/ly/gimbal/angles──► [detector]
                                           │
                              /ly/detector/armors (Armors.msg)
                                           │
                                    [tracker_solver]
                                           │
                              /ly/tracker/results (Trackers.msg)
                                           │
                                      [predictor]
                                           │
                              /ly/predictor/target (Target.msg)
                                           │
                                    [behavior_tree]
```

---

## 修改注意事項

- **新增消息字段**：在 `.msg` 文件中添加後，需要重新 `colcon build --packages-select auto_aim_common`，所有依賴此包的節點都需要重新編譯
- **修改 `DetectionType.hpp`**：會同時影響 `detector`、`tracker_solver`、`outpost_hitter` 的轉換邏輯
- **修改 `Target.msg`**：需要同步修改 `predictor`、`outpost_hitter`、`buff_hitter`（發布端）和 `behavior_tree`（接收端）
