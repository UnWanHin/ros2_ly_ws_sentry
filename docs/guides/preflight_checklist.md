# 🚗 上車前最終檢查清單

## ✅ 已完成項目

### 1. 編譯狀態
- ✅ 所有包都能單獨編譯成功
- ✅ auto_aim_common, detector, tracker_solver, predictor, gimbal_driver
- ✅ shooting_table_calib, buff_hitter, outpost_hitter

### 2. Launch 檔案
已創建 3 個測試 launch 檔案：

#### [`src/detector/launch/auto_aim.launch.py`](../../src/detector/launch/auto_aim.launch.py)
- **用途**: 裝甲板自瞄測試
- **啟動節點**: gimbal_driver + detector + tracker_solver + predictor
- **啟動命令**: `ros2 launch detector auto_aim.launch.py`

#### [`src/detector/launch/outpost.launch.py`](../../src/detector/launch/outpost.launch.py)
- **用途**: 前哨站打擊測試
- **啟動節點**: gimbal_driver + outpost_hitter
- **啟動命令**: `ros2 launch detector outpost.launch.py`

#### [`src/detector/launch/buff.launch.py`](../../src/detector/launch/buff.launch.py)
- **用途**: 能量機關打擊測試
- **啟動節點**: gimbal_driver + buff_hitter
- **啟動命令**: `ros2 launch detector buff.launch.py`

### 3. 單獨火控測試腳本
- ✅ [`mapper_node.py`](../../src/detector/script/mapper_node.py) - `Target -> /ly/control/angles + /ly/control/firecode`
- ✅ [`fire_flip_test.py`](../../src/detector/script/fire_flip_test.py) - 純火控翻轉壓測（不依賴目標）
- ✅ 測試按需單獨啟動，不和正式 BT 鏈路耦合

### 4. 配置文件統一
- ✅ 所有 launch 檔案都使用統一配置：[`src/detector/config/auto_aim_config.yaml`](../../src/detector/config/auto_aim_config.yaml)
- ✅ 路徑使用 `$HOME` 環境變量，避免硬編碼

### 5. 路徑修復
- ✅ [`shooting_table_node.cpp:433`](../../src/shooting_table_calib/shooting_table_node.cpp:433) - 使用 `$HOME/workspace/record`
- ✅ [`buff_hitter/config/config.json:15`](../../src/buff_hitter/config/config.json:15) - 使用相對路徑 `src/buff_hitter/utils/buff_models/red_best.xml`

---

## ⚠️ 上車前必須修改的配置

### 🔴 關鍵配置修改

**文件**: [`src/detector/config/auto_aim_config.yaml`](../../src/detector/config/auto_aim_config.yaml)

#### 1. 圖像源（第 45 行）
```yaml
# 當前（測試用）：
"detector_config/use_video": true

# 上車前必須改為：
"detector_config/use_video": false
```
**原因**: `use_video: true` 是從視頻文件讀取，實機必須從相機讀取。

#### 2. 虛擬設備（第 110 行）
```yaml
# 測試時（無下位機）：
io_config:
  use_virtual_device: true

# 上車前必須改為：
io_config:
  use_virtual_device: false
```
**原因**: `use_virtual_device: true` 是模擬串口，實機必須連接真實下位機。

---

## 📋 消息通訊接口檢查

### Topic 通訊鏈路

#### 快速火控測試（mapper_node / fire_flip_test）

**自瞄模式**:
```
相機 → gimbal_driver → detector → tracker_solver → predictor
     → /ly/predictor/target → mapper_node（單獨啟動）
     → /ly/control/angles + /ly/control/firecode → gimbal_driver
     → 下位機（開火）
```

**前哨模式**:
```
相機 → gimbal_driver → outpost_hitter
     → /ly/outpost/target → mapper_node（按需改訂閱）
     → /ly/control/angles + /ly/control/firecode → gimbal_driver
     → 下位機（開火）
```

**能量機關模式**:
```
相機 → gimbal_driver → buff_hitter
     → /ly/buff/target → mapper_node（按需改訂閱）
     → /ly/control/angles + /ly/control/firecode → gimbal_driver
     → 下位機（開火）
```

#### 正常模式（behavior_tree）

需要額外啟動 behavior_tree：
```
相機 → gimbal_driver → detector → tracker_solver → predictor
     → /ly/predictor/target → behavior_tree（決策）
     → /ly/control/angles + /ly/control/firecode → gimbal_driver
     → 下位機（智能開火）
```

### 消息類型
- `/ly/detector/armors` - [`auto_aim_common::msg::Armors`](../../src/auto_aim_common/msg/Armors.msg)
- `/ly/tracker/results` - [`auto_aim_common::msg::Trackers`](../../src/auto_aim_common/msg/Trackers.msg)
- `/ly/predictor/target` - [`auto_aim_common::msg::Target`](../../src/auto_aim_common/msg/Target.msg)
- `/ly/gimbal/angles` - [`gimbal_driver::msg::GimbalAngles`](../../src/gimbal_driver/msg/GimbalAngles.msg)

---

## 🕐 時間同步檢查

### ROS2 時間戳使用
所有節點都使用 `rclcpp::Time` 和 `builtin_interfaces::msg::Time`：

- ✅ [`detector_node.cpp`](../../src/detector/detector_node.cpp) - 使用 `this->now()`
- ✅ [`tracker_solver`](../../src/tracker_solver/car_tracker_solver_node.cpp) - 使用 `msg->header.stamp`
- ✅ [`predictor`](../../src/predictor/predictor_node.cpp) - 使用 `msg->header.stamp`
- ✅ [`gimbal_driver`](../../src/gimbal_driver/main.cpp) - 使用 `rclcpp::Time`

**注意**: ROS2 默認使用系統時間，確保車載電腦時間已同步（NTP 或手動設置）。

---

## 🧪 上車測試步驟

### 1. 編譯確認
```bash
cd ~/ros2_ly_ws_sentary
colcon build
source install/setup.bash
```

### 2. 修改配置
```bash
# 編輯配置文件
nano src/detector/config/auto_aim_config.yaml

# 修改第 45 行：
"detector_config/use_video": false
```

### 3. 測試自瞄模式
```bash
ros2 launch detector auto_aim.launch.py
```

**預期輸出**:
- 相機畫面正常
- 檢測到裝甲板時有框選
- `/ly/detector/armors` 有數據發布
- `/ly/gimbal/angles` 有角度輸出

### 4. 測試前哨模式
```bash
ros2 launch detector outpost.launch.py
```
可選：另起終端執行 `python3 src/detector/script/fire_flip_test.py --fire-hz 8.0`

### 5. 測試能量機關模式
```bash
ros2 launch detector buff.launch.py
```
可選：另起終端執行 `python3 src/detector/script/fire_flip_test.py --fire-hz 8.0`

### 6. 檢查 Topic 通訊
```bash
# 查看所有 topic
ros2 topic list

# 監聽關鍵 topic
ros2 topic echo /ly/detector/armors
ros2 topic echo /ly/gimbal/angles
```

---

## 🔧 故障排查

### 問題 1: 相機無法打開
**檢查**:
- 相機 USB 連接
- 相機驅動是否安裝（GalaxySDK）
- 配置文件中 `use_video: false`

### 問題 2: 沒有檢測輸出
**檢查**:
- 模型文件路徑是否正確
- OpenVINO 是否正確安裝
- 光線條件是否足夠

### 問題 3: Topic 沒有數據
**檢查**:
```bash
ros2 node list  # 確認節點都在運行
ros2 topic hz /ly/detector/armors  # 檢查發布頻率
```

### 問題 4: 時間戳異常
**檢查**:
```bash
date  # 確認系統時間
ros2 topic echo /ly/detector/armors --field header.stamp
```

---

## 📝 配置文件位置總結

### 主配置
- [`src/detector/config/auto_aim_config.yaml`](../../src/detector/config/auto_aim_config.yaml) - **統一配置文件**

### Launch 檔案
- [`src/detector/launch/auto_aim.launch.py`](../../src/detector/launch/auto_aim.launch.py)
- [`src/detector/launch/outpost.launch.py`](../../src/detector/launch/outpost.launch.py)
- [`src/detector/launch/buff.launch.py`](../../src/detector/launch/buff.launch.py)

### 其他配置
- [`src/buff_hitter/config/config.json`](../../src/buff_hitter/config/config.json) - buff 專用配置

---

## ✅ 最終確認清單

上車前請確認：

- [ ] 所有包編譯成功
- [ ] **`auto_aim_config.yaml` 第 45 行改為 `use_video: false`**
- [ ] 相機已連接並測試
- [ ] 系統時間已同步
- [ ] 測試過至少一個 launch 檔案
- [ ] 確認 `/ly/gimbal/angles` 有輸出
- [ ] 下位機串口通訊正常

---

## 🎯 核心提醒

**最關鍵的一步**: 
```yaml
"detector_config/use_video": false  # 必須改為 false！
```

不改這個配置，系統會嘗試從視頻文件讀取而不是相機，導致無法工作。

---

**準備好了就上車吧！祝比賽順利！🏆**
