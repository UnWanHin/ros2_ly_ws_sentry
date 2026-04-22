# 系統行為說明（最終版）

## ❓ 問題：現在 launch 後會自動打嗎？

### 📋 答案：取決於是否啟動「控制端」節點

目前倉庫有兩種控制來源：
- 測試控制：`mapper_node.py` / `fire_flip_test.py`
- 比賽控制：`behavior_tree`

---

## 🔍 兩種工作模式

### 模式 1: 快速測試（mapper_node / fire_flip_test）

**啟動**:
```bash
ros2 launch detector auto_aim.launch.py
python3 src/detector/script/mapper_node.py --target-id 6 --enable-fire true --auto-fire true
# 或 python3 src/detector/script/fire_flip_test.py --fire-hz 8.0
```

**啟動的節點**:
- gimbal_driver
- detector
- tracker_solver
- predictor
- **mapper_node / fire_flip_test** (測試控制)

**結果**: ✅ **會自動打（看到就打）**

**原因**:
- predictor 發布 `/ly/predictor/target`
- `mapper_node` 轉發到 `/ly/control/angles` 和 `/ly/control/firecode`
- gimbal_driver 訂閱 `/ly/control/angles` 和 `/ly/control/firecode`
- **直接發送到下位機開火**

---

### 模式 2: 正常模式（behavior_tree）

**啟動**:
```bash
# 終端 1: 感知模塊
ros2 launch detector auto_aim.launch.py

# 終端 2: 決策模塊
ros2 launch behavior_tree behavior_tree.launch.py
```

**啟動的節點**:
- gimbal_driver
- detector
- tracker_solver
- predictor
- **behavior_tree** (決策模塊)

**結果**: ✅ **會智能決策後開火**

**原因**:
- behavior_tree 接管決策，根據遊戲狀態決定是否開火

---

### 情況 2: 啟動 behavior_tree.launch.py（原有的 launch）

```bash
ros2 launch behavior_tree behavior_tree.launch.py
```

**啟動的節點**:
- behavior_tree

**behavior_tree 的作用** ([`src/behavior_tree/src/SubscribeMessage.cpp:166-174`](../../src/behavior_tree/src/SubscribeMessage.cpp:166)):

```cpp
// 訂閱 predictor 的目標
GenSub<ly_predictor_target>([](Application& app, auto msg) {
    obj.autoAimData.Angles.Yaw = msg->yaw;
    obj.autoAimData.Angles.Pitch = msg->pitch;
    obj.autoAimData.FireStatus = true;  // ← 設置射擊狀態
    obj.isFindTargetAtomic = true;
});
```

**behavior_tree 發布控制指令** ([`src/behavior_tree/src/PublishMessage.cpp:52-66`](../../src/behavior_tree/src/PublishMessage.cpp:52)):

```cpp
void Application::PubGimbalControlData() {
    // 發布角度
    using topic = ly_control_angles;
    topic::Msg msg;
    msg.Yaw = gimbalControlData.GimbalAngles.Yaw;
    msg.Pitch = gimbalControlData.GimbalAngles.Pitch;
    node.Publisher<topic>().publish(msg);
    
    // 發布射擊指令
    using topic = ly_control_firecode;
    topic::Msg msg;
    msg.data = *reinterpret_cast<std::uint8_t *>(&gimbalControlData.FireCode);
    node.Publisher<topic>().publish(msg);
}
```

**結果**: ✅ **會自動打**（如果 behavior_tree 邏輯允許）

---

## 🎯 完整的系統架構

### 正確的啟動方式

**需要同時啟動兩個 launch**:

```bash
# 終端 1: 啟動感知和計算模塊
ros2 launch detector auto_aim.launch.py

# 終端 2: 啟動決策模塊
ros2 launch behavior_tree behavior_tree.launch.py
```

### 完整的消息流

```
相機
 ↓
gimbal_driver (發布雲台角度)
 ↓
detector (檢測裝甲板)
 ↓ /ly/detector/armors
tracker_solver (追蹤)
 ↓ /ly/tracker/results
predictor (預測)
 ↓ /ly/predictor/target
behavior_tree (決策) ← 這是關鍵！
 ↓ /ly/control/angles (角度)
 ↓ /ly/control/firecode (射擊)
gimbal_driver (發送下位機)
 ↓
下位機
```

---

## 🚨 你創建的 launch 檔案的問題

### 問題：缺少 behavior_tree

你創建的三個 launch 檔案：
- [`auto_aim.launch.py`](../../src/detector/launch/auto_aim.launch.py)
- [`outpost.launch.py`](../../src/detector/launch/outpost.launch.py)
- [`buff.launch.py`](../../src/detector/launch/buff.launch.py)

**都沒有啟動 behavior_tree！**

### 解決方案

#### 方案 1: 修改 launch 檔案，添加 behavior_tree

```python
# auto_aim.launch.py
return LaunchDescription([
    Node(package='gimbal_driver', ...),
    Node(package='detector', ...),
    Node(package='tracker_solver', ...),
    Node(package='predictor', ...),
    Node(package='behavior_tree',  # ← 添加這個
         executable='behavior_tree_node',
         name='behavior_tree',
         output='screen'),
])
```

#### 方案 2: 分開啟動（當前方式）

```bash
# 終端 1
ros2 launch detector auto_aim.launch.py

# 終端 2
ros2 launch behavior_tree behavior_tree.launch.py
```

---

## 📊 behavior_tree 的決策邏輯

### behavior_tree 訂閱的 Topic

**感知數據**:
- `/ly/predictor/target` - 自瞄目標
- `/ly/buff/target` - 能量機關目標
- `/ly/outpost/target` - 前哨目標
- `/ly/detector/armors` - 檢測結果

**遊戲狀態**:
- `/ly/game/is_start` - 比賽是否開始
- `/ly/me/hp` - 我方血量
- `/ly/enemy/hp` - 敵方血量
- `/ly/me/ammo_left` - 剩餘彈藥
- `/ly/me/is_at_home` - 是否在家

### behavior_tree 發布的 Topic

**控制指令**:
- `/ly/control/angles` - 雲台角度
- `/ly/control/firecode` - 射擊指令
- `/ly/control/vel` - 底盤速度

**模式控制**:
- `/ly/bt/target` - 目標選擇
- `/ly/aa/enable` - 自瞄使能
- `/ly/ra/enable` - 能量機關使能
- `/ly/outpost/enable` - 前哨使能

---

## ⚠️ 重要結論

### 你的 launch 檔案

**現狀**: 
- ❌ 只啟動了感知和計算模塊
- ❌ 沒有啟動決策模塊 (behavior_tree)
- ❌ **不會自動打**

**需要修改**:
1. 在 launch 檔案中添加 behavior_tree 節點
2. 或者手動啟動 behavior_tree

### 完整系統需要的節點

**自瞄模式**:
```
gimbal_driver + detector + tracker_solver + predictor + behavior_tree
```

**前哨模式**:
```
gimbal_driver + outpost_hitter + behavior_tree
```

**能量機關模式**:
```
gimbal_driver + buff_hitter + behavior_tree
```

---

## 🔧 建議的修改

### 更新 auto_aim.launch.py

```python
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    home_dir = os.environ['HOME']
    config_file = os.path.join(home_dir, 'ros2_ly_ws_sentary/src/detector/config/auto_aim_config.yaml')
    
    return LaunchDescription([
        Node(package='gimbal_driver', executable='gimbal_driver_node', 
             name='gimbal_driver', output='screen', parameters=[config_file]),
        Node(package='detector', executable='detector_node',
             name='detector', output='screen', parameters=[config_file]),
        Node(package='tracker_solver', executable='tracker_solver_node',
             name='tracker_solver', output='screen', parameters=[config_file]),
        Node(package='predictor', executable='predictor_node',
             name='predictor_node', output='screen', parameters=[config_file]),
        Node(package='behavior_tree', executable='behavior_tree_node',  # ← 添加
             name='behavior_tree', output='screen', emulate_tty=True),
    ])
```

---

## 📝 測試方法

### 測試當前系統（沒有 behavior_tree）

```bash
# 啟動
ros2 launch detector auto_aim.launch.py

# 監聽 topic
ros2 topic echo /ly/predictor/target      # 有數據
ros2 topic echo /ly/control/angles        # 沒有數據 ← 問題
```

### 測試完整系統（有 behavior_tree）

```bash
# 終端 1
ros2 launch detector auto_aim.launch.py

# 終端 2
ros2 launch behavior_tree behavior_tree.launch.py

# 終端 3: 監聽
ros2 topic echo /ly/predictor/target      # 有數據
ros2 topic echo /ly/control/angles        # 有數據 ← 正常
ros2 topic echo /ly/control/firecode      # 有數據 ← 會射擊
```

---

## ✅ 最終答案

### 你的問題：launch 後會自動打嗎？

**答案**: 
- ❌ 用你創建的 launch 檔案 → **不會打**（缺少 behavior_tree）
- ✅ 添加 behavior_tree 後 → **會打**（完整系統）

### 需要做的事

1. **修改三個 launch 檔案**，添加 behavior_tree 節點
2. 或者**手動啟動兩個 launch**（感知 + 決策）
3. 確保 behavior_tree 編譯成功（目前編譯失敗）

---

**總結**: 你的 launch 檔案是"半成品"，只有感知沒有決策，需要添加 behavior_tree 才能完整工作。
