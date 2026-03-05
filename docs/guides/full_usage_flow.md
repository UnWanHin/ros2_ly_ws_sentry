# 🚀 ROS2 哨兵機器人完整使用流程

## 📋 系統狀態

✅ **所有包編譯成功**  
✅ **消息通訊鏈路完整**  
✅ **單獨火控測試腳本已提供**

---

## 🎯 兩種工作模式

### 模式 1: 快速火控測試（看到就打）

**適用場景**: 快速功能測試，無需決策模塊

**啟動方式**:
```bash
cd ~/ros2_ly_ws_sentary
source install/setup.bash

# 終端 1: 啟動檢測鏈路（自瞄/前哨/打符三選一）
ros2 launch detector auto_aim.launch.py

# 終端 2: 啟動測試控制（有目標才翻轉開火）
python3 src/detector/script/mapper_node.py --target-id 6 --enable-fire true --auto-fire true

# 或純火控翻轉壓測（不依賴目標）
python3 src/detector/script/fire_flip_test.py --fire-hz 8.0
```

**工作流程**:
```
相機 → detector/outpost/buff → mapper_node / fire_flip_test → gimbal_driver → 下位機
                                           ↓
                                   測試控制（翻轉開火）
```

**特點**:
- ✅ 檢測到有效目標立即開火
- ✅ 無需 behavior_tree 決策
- ✅ 適合快速測試瞄準和開火功能
- ⚠️ 沒有智能決策（不考慮血量、彈藥等）

---

### 模式 2: 正常模式（智能決策）

**適用場景**: 正式比賽，需要完整決策邏輯

**啟動方式**:
```bash
cd ~/ros2_ly_ws_sentary
source install/setup.bash

# 終端 1: 啟動感知模塊
ros2 launch detector auto_aim.launch.py

# 終端 2: 啟動決策模塊
ros2 launch behavior_tree behavior_tree.launch.py
```

**工作流程**:
```
相機 → detector → tracker → predictor → behavior_tree → gimbal_driver → 下位機
                                              ↓
                                    智能決策（考慮遊戲狀態）
```

**特點**:
- ✅ 根據血量、彈藥、遊戲狀態決策
- ✅ 支持目標優先級選擇
- ✅ 支持模式自動切換（自瞄/前哨/能量機關）
- ⚠️ 需要 behavior_tree 決策模塊完整實現

---

## 🔧 上車前配置檢查

### 必須修改的參數

**文件**: [`src/detector/config/auto_aim_config.yaml`](../../src/detector/config/auto_aim_config.yaml)

#### 1. 圖像源（第 45 行）
```yaml
# 測試時（使用視頻）
"detector_config/use_video": true

# 上車時（使用相機）
"detector_config/use_video": false  # ← 必須改為 false
```

#### 2. 虛擬設備（第 110 行）
```yaml
# 測試時（無下位機）
io_config:
  use_virtual_device: true

# 上車時（連接下位機）
io_config:
  use_virtual_device: false  # ← 必須改為 false
```

---

## 🎮 控制下位機的機制

### 開火指令流程

**mapper_node.py** ([`src/detector/script/mapper_node.py:83`](../../src/detector/script/mapper_node.py:83)):
```python
# 有目標時翻轉 0b00/0b11 以觸發連續開火
self.fire_status = 0b11 if self.fire_status == 0 else 0b00
fire_msg = UInt8()
fire_msg.data = self.fire_status
self.firecode_pub.publish(fire_msg)
```

**gimbal_driver** ([`src/gimbal_driver/main.cpp:90`](../../src/gimbal_driver/main.cpp:90)):
```cpp
// 訂閱開火指令
GenSub<ly_control_firecode>([](GimbalControlData& g, const std_msgs::msg::UInt8& m) {
    *reinterpret_cast<std::uint8_t*>(&g.FireCode) = m.data;
});

// 發送到下位機 (line 358)
Device.Write(data);  // 通過串口發送
```

### 角度控制流程

**mapper_node.py** ([`src/detector/script/mapper_node.py:61`](../../src/detector/script/mapper_node.py:61)):
```python
gimbal_msg = GimbalAngles()
gimbal_msg.header = msg.header
gimbal_msg.yaw = msg.yaw
gimbal_msg.pitch = msg.pitch
self.gimbal_pub.publish(gimbal_msg)
```

**gimbal_driver** ([`src/gimbal_driver/main.cpp:84`](../../src/gimbal_driver/main.cpp:84)):
```cpp
// 訂閱角度指令
GenSub<ly_control_angles>([](GimbalControlData& g, const gimbal_driver::msg::GimbalAngles& m) {
    g.GimbalAngles.Yaw = static_cast<float>(m.yaw);
    g.GimbalAngles.Pitch = static_cast<float>(m.pitch);
});

// 發送到下位機
Device.Write(data);  // 通過串口發送
```

**結論**: ✅ **是的，直接控制下位機的雲台和開火**

---

## 📊 消息通訊鏈路

### 自瞄模式完整鏈路

```
相機硬件
  ↓
gimbal_driver (讀取圖像 + 雲台角度)
  ↓ 發布: /ly/gimbal/angles
detector (檢測裝甲板)
  ↓ 發布: /ly/detector/armors
tracker_solver (追蹤)
  ↓ 發布: /ly/tracker/results
predictor (預測 + 彈道解算)
  ↓ 發布: /ly/predictor/target
  
【測試鏈路】
mapper_node / fire_flip_test
  ↓ 發布: /ly/control/angles
  ↓ 發布: /ly/control/firecode
  
【比賽鏈路】
behavior_tree
  ↓ 發布: /ly/control/angles
  ↓ 發布: /ly/control/firecode
  
gimbal_driver (接收控制指令)
  ↓ 串口通訊
下位機 (執行雲台控制 + 開火)
```

### 前哨模式完整鏈路

```
相機硬件
  ↓
gimbal_driver
  ↓
outpost_hitter (檢測 + 預測 + 解算)
  ↓ 發布: /ly/outpost/target
mapper_node（測試） / behavior_tree（比賽）
  ↓ 發布: /ly/control/angles + /ly/control/firecode
gimbal_driver
  ↓
下位機
```

### 能量機關模式完整鏈路

```
相機硬件
  ↓
gimbal_driver
  ↓
buff_hitter (檢測 + 預測 + 解算)
  ↓ 發布: /ly/buff/target
mapper_node（測試） / behavior_tree（比賽）
  ↓ 發布: /ly/control/angles + /ly/control/firecode
gimbal_driver
  ↓
下位機
```

---

## 🔄 模式切換方法

### 從快速測試切換到正常模式

1. **停止測試節點**:
```bash
pkill -f mapper_node
pkill -f fire_flip_test
```

2. **啟動感知鏈路**:
```bash
ros2 launch detector auto_aim.launch.py
```

3. **啟動決策模塊**:
```bash
ros2 launch behavior_tree behavior_tree.launch.py
```

### 從正常模式切換到快速測試

1. **停止 behavior_tree**:
```bash
pkill -f behavior_tree_node
```

2. **啟動感知鏈路 + 測試節點**:
```bash
ros2 launch detector auto_aim.launch.py
python3 src/detector/script/mapper_node.py --target-id 6 --enable-fire true --auto-fire true
```

---

## ✅ 編譯和啟動命令

### 首次編譯

```bash
cd ~/ros2_ly_ws_sentary

# 編譯所有包
colcon build

# 或單獨編譯
colcon build --packages-select detector gimbal_driver tracker_solver predictor outpost_hitter buff_hitter

# 加載環境
source install/setup.bash
```

### 修改代碼後重新編譯

```bash
cd ~/ros2_ly_ws_sentary

# 只編譯修改的包（例如 detector）
colcon build --packages-select detector

# 加載環境
source install/setup.bash
```

### 啟動系統

**測試模式**:
```bash
ros2 launch detector auto_aim.launch.py
python3 src/detector/script/mapper_node.py --target-id 6 --enable-fire true --auto-fire true
```

**正常模式**:
```bash
# 終端 1
ros2 launch detector auto_aim.launch.py

# 終端 2
ros2 launch behavior_tree behavior_tree.launch.py
```

---

## 🐛 調試命令

### 查看 Topic 列表
```bash
ros2 topic list
```

### 查看消息內容
```bash
# 查看檢測結果
ros2 topic echo /ly/detector/armors

# 查看預測目標
ros2 topic echo /ly/predictor/target

# 查看控制指令
ros2 topic echo /ly/control/angles
ros2 topic echo /ly/control/firecode
```

### 查看節點狀態
```bash
ros2 node list
ros2 node info /target_to_gimbal_mapper
```

### 查看參數
```bash
ros2 param list /target_to_gimbal_mapper
ros2 param get /target_to_gimbal_mapper use_sim_time
```

---

## 📚 相關文檔

- [`消息通訊鏈路文檔.md`](消息通訊鏈路文檔.md) - 詳細的消息流程
- [`上車前最終檢查清單.md`](上車前最終檢查清單.md) - 部署前檢查項目
- [`系統行為說明_更新.md`](系統行為說明_更新.md) - 系統行為詳解
- [`TEST_GUIDE.md`](TEST_GUIDE.md) - 測試指南
- [`CONFIG_SETUP_GUIDE.md`](CONFIG_SETUP_GUIDE.md) - 配置指南

---

## ⚠️ 重要提醒

1. **上車前必須修改**:
   - `use_video: false` (使用相機)
   - `use_virtual_device: false` (連接下位機)

2. **測試時注意**:
   - `mapper_node`/`fire_flip_test` 會輸出開火翻轉，注意安全
   - 確保下位機已連接（或使用虛擬設備測試）

3. **決策模塊**:
   - 正式比賽需要啟動 behavior_tree
   - behavior_tree 需要完整實現才能正常工作

4. **串口權限**:
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```
