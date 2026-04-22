# ROS2 哨兵機器人消息通訊鏈路文檔

## 📋 文檔說明

本文檔梳理現有系統的消息通訊鏈路，從數據獲取到發送下位機的完整流程。
**注意**: 当前實際主鏈路包含 `behavior_tree`，若不啟動決策節點需使用橋接節點把 `Target` 類消息轉成 `/ly/control/*`。

---

## 🎯 三種工作模式

系統支持三種工作模式，每種模式有獨立的消息鏈路：

1. **自瞄模式 (Auto-Aim)** - 裝甲板識別與追蹤
2. **前哨模式 (Outpost)** - 前哨站打擊
3. **能量機關模式 (Buff)** - 能量機關打擊

---

## 🔄 模式 1: 自瞄模式 (Auto-Aim)

### 數據流向圖

```
相機硬件
   ↓
[gimbal_driver] ← 從下位機接收雲台角度
   ↓ 發布圖像
   ↓
[detector] ← 訂閱: /ly/gimbal/angles (雲台角度)
   ↓ 檢測裝甲板
   ↓ 發布: /ly/detector/armors
   ↓
[tracker_solver] ← 訂閱: /ly/detector/armors
   ↓ 追蹤與位姿解算
   ↓ 發布: /ly/tracker/results
   ↓
[predictor] ← 訂閱: /ly/tracker/results
   ↓ 預測與彈道解算
   ↓ 發布: /ly/predictor/target
   ↓
[behavior_tree] ← 訂閱: /ly/predictor/target
   ↓ 發布: /ly/control/angles + /ly/control/firecode (+ /ly/control/vel,/ly/control/posture)
   ↓
[gimbal_driver] ← 訂閱: /ly/control/*
   ↓ 串口下發主控制幀
   ↓
下位機 (串口通訊)
```

### 詳細消息鏈路

#### 1. 數據獲取 - gimbal_driver

**文件**: [`src/gimbal_driver/main.cpp`](../../src/gimbal_driver/main.cpp)

**數據來源**:
- 相機硬件 (GalaxySDK) 或視頻文件
- 下位機串口數據 (雲台角度、遊戲狀態等)

**發布的 Topic**:
- `/ly/gimbal/angles` - 雲台當前角度
  - 類型: [`gimbal_driver::msg::GimbalAngles`](../../src/gimbal_driver/msg/GimbalAngles.msg)
  - 內容: `yaw`, `pitch`
  
- `/ly/me/is_team_red` - 我方隊伍顏色
  - 類型: `std_msgs::msg::Bool`
  - 內容: `true` = 紅方, `false` = 藍方

- `/ly/bullet/speed` - 子彈速度
  - 類型: `std_msgs::msg::Float32`
  - 內容: 當前子彈速度 (m/s)

**訂閱的 Topic** (用於接收控制指令):
- `/ly/control/angles` - 控制雲台角度
  - 類型: [`gimbal_driver::msg::GimbalAngles`](../../src/gimbal_driver/msg/GimbalAngles.msg)

**關鍵代碼位置**:
```cpp
// 定義 Topic (第 29-62 行)
LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
LY_DEF_ROS_TOPIC(ly_me_is_team_red, "/ly/me/is_team_red", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::msg::Float32);
```

---

#### 2. 裝甲板檢測 - detector

**文件**: [`src/detector/detector_node.cpp`](../../src/detector/detector_node.cpp)

**訂閱的 Topic**:
- `/ly/gimbal/angles` - 雲台角度 (用於 PnP 解算)
- `/ly/me/is_team_red` - 隊伍顏色 (用於顏色過濾)
- `/ly/bt/target` - 目標選擇 (決策模塊發送，可選)
- `/ly/aa/enable` - 自瞄使能開關

**發布的 Topic**:
- `/ly/detector/armors` - 檢測到的裝甲板列表
  - 類型: [`auto_aim_common::msg::Armors`](../../src/auto_aim_common/msg/Armors.msg)
  - 內容:
    - `header.stamp` - 時間戳
    - `yaw`, `pitch` - 當前雲台角度
    - `armors[]` - 裝甲板數組
      - `type` - 裝甲板類型 (0-6: 不同車輛)
      - `corners_x[]`, `corners_y[]` - 四個角點座標
      - `distance` - 距離
    - `cars[]` - 車輛檢測框

**處理流程**:
1. 從相機獲取圖像 (第 321-338 行)
2. 使用 OpenVINO 模型檢測裝甲板 (第 358 行)
3. 顏色過濾 (第 369-378 行)
4. PnP 位姿解算 (第 380 行)
5. 發布檢測結果 (第 389 行)

**關鍵代碼位置**:
```cpp
// 發布檢測結果 (第 388-392 行)
if(aa_enable){
    global_node->Publisher<ly_detector_armors>()->publish(armor_list_msg);
}
```

---

#### 3. 追蹤與位姿解算 - tracker_solver

**文件**: [`src/tracker_solver/car_tracker_solver_node.cpp`](../../src/tracker_solver/car_tracker_solver_node.cpp)

**訂閱的 Topic**:
- `/ly/detector/armors` - 檢測到的裝甲板

**發布的 Topic**:
- `/ly/tracker/results` - 追蹤結果
  - 類型: [`auto_aim_common::msg::Trackers`](../../src/auto_aim_common/msg/Trackers.msg)
  - 內容:
    - `header.stamp` - 時間戳
    - `yaw`, `pitch` - 雲台角度
    - `armor_trackers[]` - 裝甲板追蹤器
      - `car_id` - 車輛 ID
      - `armor_id` - 裝甲板 ID
      - `x`, `y`, `z` - 3D 位置 (相機座標系)
      - `yaw` - 裝甲板朝向
    - `car_trackers[]` - 車輛追蹤器

**處理流程**:
1. 接收檢測結果 (第 132 行)
2. 數據關聯與追蹤 (第 154-155 行)
3. 3D 位姿解算 (第 169 行)
4. 發布追蹤結果 (第 173 行)

**關鍵代碼位置**:
```cpp
// 訂閱檢測結果 (第 67-69 行)
node.GenSubscriber<ly_detector_armors>([this](const auto_aim_common::msg::Armors::ConstSharedPtr msg) { 
    detection_callback(msg); 
});

// 發布追蹤結果 (第 173 行)
node.Publisher<ly_tracker_results>()->publish(trackers_msg);
```

---

#### 4. 預測與彈道解算 - predictor

**文件**: [`src/predictor/predictor_node.cpp`](../../src/predictor/predictor_node.cpp)

**訂閱的 Topic**:
- `/ly/tracker/results` - 追蹤結果
- `/ly/bt/target` - 目標選擇 (決策模塊發送，可選)
- `/ly/bullet/speed` - 子彈速度

**發布的 Topic**:
- `/ly/predictor/target` - 預測目標
  - 類型: [`auto_aim_common::msg::Target`](../../src/auto_aim_common/msg/Target.msg)
  - 內容:
    - `header.stamp` - 時間戳
    - `status` - 是否有效目標 (bool)
    - `yaw` - 目標 yaw 角
    - `pitch` - 目標 pitch 角

- `/ly/predictor/debug` - 調試信息 (EKF 狀態)
  - 類型: [`auto_aim_common::msg::DebugFilter`](../../src/auto_aim_common/msg/DebugFilter.msg)

**處理流程**:
1. 接收追蹤結果 (第 106 行)
2. 目標選擇 (第 119 行)
3. EKF 預測 (第 126 行)
4. 彈道解算 (第 119 行)
5. 發布控制指令 (第 153 行)

**關鍵代碼位置**:
```cpp
// 訂閱追蹤結果 (第 59-61 行)
node.GenSubscriber<ly_tracker_results>([this](const auto_aim_common::msg::Trackers::ConstSharedPtr msg) { 
    predictor_callback(msg); 
});

// 發布預測目標 (第 153-154 行)
if(control_result.valid){
    node.Publisher<ly_predictor_target>()->publish(target_msg);
}
```

---

#### 5. 決策與下發 - behavior_tree + gimbal_driver

**文件**:
- [`src/behavior_tree/src/SubscribeMessage.cpp`](../../src/behavior_tree/src/SubscribeMessage.cpp)
- [`src/behavior_tree/src/PublishMessage.cpp`](../../src/behavior_tree/src/PublishMessage.cpp)
- [`src/gimbal_driver/main.cpp`](../../src/gimbal_driver/main.cpp)

**behavior_tree 訂閱**:
- `/ly/predictor/target` - 預測目標 (自瞄模式)
- `/ly/outpost/target` - 前哨目標 (前哨模式)
- `/ly/buff/target` - 能量機關目標 (buff 模式)

**behavior_tree 發布**:
- `/ly/control/angles`
- `/ly/control/firecode`
- `/ly/control/vel`
- `/ly/control/posture`

**gimbal_driver 訂閱**:
- `/ly/control/*`

**輸出**:
- 串口通訊發送到下位機主控制幀 `GimbalControlData`

---

## 🔄 模式 2: 前哨模式 (Outpost)

### 數據流向圖

```
相機硬件
   ↓
[gimbal_driver] ← 從下位機接收雲台角度
   ↓ 發布圖像
   ↓
[detector] ← 訂閱: /ly/gimbal/angles
   ↓ 檢測前哨站裝甲板
   ↓ 發布: /ly/outpost/armors
   ↓
[outpost_hitter] ← 訂閱: /ly/outpost/armors
   ↓ 前哨站預測與解算
   ↓ 發布: /ly/outpost/target
   ↓
[behavior_tree] ← 訂閱: /ly/outpost/target
   ↓ 發布: /ly/control/*
   ↓
[gimbal_driver] ← 訂閱: /ly/control/*
   ↓ 串口下發主控制幀
   ↓
下位機 (串口通訊)
```

### 詳細消息鏈路

#### 1. 前哨站檢測 - detector

**發布的 Topic**:
- `/ly/outpost/armors` - 前哨站裝甲板
  - 類型: [`auto_aim_common::msg::Armors`](../../src/auto_aim_common/msg/Armors.msg)
  - 內容: 與自瞄模式相同，但只包含前哨站裝甲板 (type=7)

**關鍵代碼位置**:
```cpp
// 發布前哨站檢測結果 (第 390-392 行)
else if(outpost_enable){
    global_node->Publisher<ly_outpost_armors>()->publish(armor_list_msg);
}
```

---

#### 2. 前哨站打擊 - outpost_hitter

**文件**: [`src/outpost_hitter/outpost_hitter_node.cpp`](../../src/outpost_hitter/outpost_hitter_node.cpp)

**訂閱的 Topic**:
- `/ly/outpost/armors` - 前哨站裝甲板檢測

**發布的 Topic**:
- `/ly/outpost/target` - 前哨站目標
  - 類型: [`auto_aim_common::msg::Target`](../../src/auto_aim_common/msg/Target.msg)
  - 內容: `status`, `yaw`, `pitch`

**處理流程**:
1. 接收前哨站檢測 (第 55 行)
2. 裝甲板過濾 (type=7) (第 197 行)
3. 位姿解算
4. 預測與選板
5. 彈道解算
6. 發布控制指令 (第 184 行)

**關鍵代碼位置**:
```cpp
// 訂閱前哨站檢測 (第 55-57 行)
node.GenSubscriber<ly_outpost_armors>([this](const auto_aim_common::msg::Armors::ConstSharedPtr msg) {
    outpost_detection_callback(msg);
});

// 發布目標 (第 184 行)
node.Publisher<ly_outpost_target>()->publish(target_msg);
```

---

## 🔄 模式 3: 能量機關模式 (Buff)

### 數據流向圖

```
相機硬件
   ↓
[gimbal_driver] ← 從下位機接收雲台角度
   ↓ 發布圖像+角度
   ↓ 發布: /ly/ra/angle_image
   ↓
[buff_hitter] ← 訂閱: /ly/ra/angle_image
   ↓ 能量機關檢測與預測
   ↓ 發布: /ly/buff/target
   ↓
[behavior_tree] ← 訂閱: /ly/buff/target
   ↓ 發布: /ly/control/*
   ↓
[gimbal_driver] ← 訂閱: /ly/control/*
   ↓ 串口下發主控制幀
   ↓
下位機 (串口通訊)
```

### 詳細消息鏈路

#### 1. 圖像發布 - gimbal_driver

**發布的 Topic**:
- `/ly/ra/angle_image` - 帶角度的圖像
  - 類型: [`auto_aim_common::msg::AngleImage`](../../src/auto_aim_common/msg/AngleImage.msg)
  - 內容:
    - `image` - 圖像數據
    - `yaw` - 當前 yaw 角
    - `pitch` - 當前 pitch 角

---

#### 2. 能量機關打擊 - buff_hitter

**文件**: [`src/buff_hitter/main.cpp`](../../src/buff_hitter/main.cpp)

**訂閱的 Topic**:
- `/ly/ra/angle_image` - 帶角度的圖像
- `/ly/ra/enable` - 能量機關使能開關
- `/ly/aa/enable` - 自瞄使能 (用於互斥)
- `/ly/bullet/speed` - 彈速回讀（動態彈道參數）

**發布的 Topic**:
- `/ly/buff/target` - 能量機關目標
  - 類型: [`auto_aim_common::msg::Target`](../../src/auto_aim_common/msg/Target.msg)
  - 內容: `status`, `yaw`, `pitch`

**處理流程**:
1. 接收圖像和角度 (第 176-187 行)
2. 能量機關檢測 (OpenVINO 模型)
3. 運動預測
4. 彈道解算
5. 發布控制指令 (第 190-198 行)

**關鍵代碼位置**:
```cpp
// 訂閱圖像 (第 176-187 行)
GenSub<ly_ra_angle_image>(
    [](RA_MultiThreadVariables &g, const auto_aim_common::msg::AngleImage &m) {
        g.GimbalAngles.pitch = static_cast<float>(m.pitch);
        g.GimbalAngles.yaw = static_cast<float>(m.yaw);
        g.Image = cv_bridge::toCvCopy(m.image, sensor_msgs::image_encodings::BGR8)->image;
    }, 1);

// 發布目標 (第 190-198 行)
void PubData(const bool& hitBuff, const gimbal_driver::msg::GimbalAngles &angles) {
    topic::Msg msg;
    msg.status = hitBuff;
    msg.yaw = static_cast<float>(angles.yaw);
    msg.pitch = static_cast<float>(angles.pitch);
    Node.Publisher<topic>()->publish(msg);
}
```

---

## 📊 消息類型定義

### 核心消息類型

#### 1. Armors - 裝甲板檢測結果
**文件**: [`src/auto_aim_common/msg/Armors.msg`](../../src/auto_aim_common/msg/Armors.msg)
```
std_msgs/Header header
float32 yaw
float32 pitch
bool is_available_armor_for_predictor
Armor[] armors
Car[] cars
```

#### 2. Trackers - 追蹤結果
**文件**: [`src/auto_aim_common/msg/Trackers.msg`](../../src/auto_aim_common/msg/Trackers.msg)
```
std_msgs/Header header
float32 yaw
float32 pitch
ArmorTracker[] armor_trackers
CarTracker[] car_trackers
```

#### 3. Target - 控制目標
**文件**: [`src/auto_aim_common/msg/Target.msg`](../../src/auto_aim_common/msg/Target.msg)
```
std_msgs/Header header
bool status
float32 yaw
float32 pitch
```

#### 4. GimbalAngles - 雲台角度
**文件**: [`src/gimbal_driver/msg/GimbalAngles.msg`](../../src/gimbal_driver/msg/GimbalAngles.msg)
```
float32 yaw
float32 pitch
```

---

## 🔧 關鍵配置

### 統一配置文件
**文件**: [`src/detector/config/auto_aim_config.yaml`](../../src/detector/config/auto_aim_config.yaml)

所有節點都從這個配置文件讀取參數：
- 相機內參
- 相機到槍口偏移
- 子彈速度
- 檢測器模型路徑
- 是否使用視頻 (`use_video`)

---

## 🎮 模式切換機制

### 使能開關 Topic

系統通過以下 Topic 控制模式切換：

1. `/ly/aa/enable` - 自瞄模式使能
   - `true` = 啟用自瞄
   - `false` = 禁用自瞄

2. `/ly/ra/enable` - 能量機關模式使能
   - `true` = 啟用能量機關
   - `false` = 禁用能量機關

3. `/ly/outpost/enable` - 前哨模式使能
   - `true` = 啟用前哨
   - `false` = 禁用前哨

**互斥邏輯**:
- 自瞄和能量機關互斥 (buff_hitter 第 238-241 行)
- detector 根據使能開關發布到不同 topic (第 388-392 行)

---

## 🕐 時間同步機制

### 時間戳傳遞

所有消息都攜帶時間戳，確保數據同步：

1. **detector** 生成時間戳:
   ```cpp
   armor_list_msg.header.stamp = global_node->now();
   ```

2. **tracker_solver** 傳遞時間戳:
   ```cpp
   trackers_msg.header.stamp = msg->header.stamp;
   ```

3. **predictor** 傳遞時間戳:
   ```cpp
   target_msg.header = msg->header;
   ```

### 時間戳轉換

為避免 "Different Time Sources" 錯誤，系統統一轉換為 double:
```cpp
double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();
Time::TimeStamp timestamp(msg_time_sec);
```

---

## 🚀 啟動命令

### 自瞄模式
```bash
ros2 launch detector auto_aim.launch.py
```
啟動節點: gimbal_driver + detector + tracker_solver + predictor

### 前哨模式
```bash
ros2 launch detector outpost.launch.py
```
啟動節點: gimbal_driver + outpost_hitter

### 能量機關模式
```bash
ros2 launch detector buff.launch.py
```
啟動節點: gimbal_driver + buff_hitter

---

## 🔍 調試 Topic

### 查看所有 Topic
```bash
ros2 topic list
```

### 監聽關鍵 Topic
```bash
# 檢測結果
ros2 topic echo /ly/detector/armors

# 追蹤結果
ros2 topic echo /ly/tracker/results

# 預測目標
ros2 topic echo /ly/predictor/target

# 雲台角度
ros2 topic echo /ly/gimbal/angles
```

### 檢查發布頻率
```bash
ros2 topic hz /ly/detector/armors
ros2 topic hz /ly/predictor/target
```

---

## 📝 決策模塊接口 (預留)

### 決策模塊需要訂閱的 Topic

1. **遊戲狀態** (來自 gimbal_driver):
   - `/ly/game/is_start` - 比賽是否開始
   - `/ly/game/time_left` - 剩餘時間
   - `/ly/me/hp` - 我方血量
   - `/ly/enemy/hp` - 敵方血量
   - `/ly/me/is_at_home` - 是否在家
   - `/ly/me/ammo_left` - 剩餘彈藥

2. **檢測結果**:
   - `/ly/detector/armors` - 裝甲板檢測
   - `/ly/tracker/results` - 追蹤結果

### 決策模塊需要發布的 Topic

1. **目標選擇**:
   - `/ly/bt/target` - 選擇攻擊目標 (ArmorType: 0-6)
   - 類型: `std_msgs::msg::UInt8`

2. **模式切換**:
   - `/ly/aa/enable` - 控制自瞄使能
   - `/ly/ra/enable` - 控制能量機關使能
   - `/ly/outpost/enable` - 控制前哨使能

---

## ⚠️ 注意事項

1. **時間同步**: 確保系統時間正確，ROS2 使用系統時間作為時間戳
2. **配置文件**: 所有節點使用統一配置 `auto_aim_config.yaml`
3. **模式互斥**: 自瞄和能量機關不能同時啟用
4. **串口通訊**: gimbal_driver 負責所有與下位機的通訊
5. **相機模式**: 實機測試時必須設置 `use_video: false`

---

## 📚 相關文件

- Launch 檔案: [`src/detector/launch/`](../../src/detector/launch/)
- 配置檔案: [`src/detector/config/auto_aim_config.yaml`](../../src/detector/config/auto_aim_config.yaml)
- 消息定義: [`src/auto_aim_common/msg/`](../../src/auto_aim_common/msg/)
- 上車檢查清單: [`上車前最終檢查清單.md`](上車前最終檢查清單.md)

---

**文檔版本**: 1.0  
**最後更新**: 2026-02-03  
**維護者**: ROS2 哨兵機器人團隊
