# buff_hitter — 打符（能量機關）節點

## 概述

`buff_hitter`（又叫 `ra_hitter`）是打擊**能量機關（Power Rune，俗稱「符」）**的專用節點。能量機關是一個移動的風扇葉片靶，命中後給我方提供增益（攻擊力/防禦力提升）。

**功能**：
1. 接收 `detector` 發布的帶雲台角的相機圖像（`AngleImage`）
2. 在圖像中識別能量機關（R標 + 靶心5點）
3. 計算打擊角度（包括旋轉預測）
4. 發布打符瞄準指令給 `behavior_tree`

---

## 目錄結構

```
buff_hitter/
├── CMakeLists.txt / package.xml
├── main.cpp                        # 主節點（Application 類）
├── config/
│   └── config.json                 # 配置（模型路徑、使能標誌等）
├── module/                         # 打符算法模塊（獨立於ROS）
│   ├── BasicTypes.hpp              # 打符相關數據類型
│   ├── BuffCalculator.hpp/.cpp     # 核心計算器（位置+旋轉預測+角度解算）
│   ├── BuffController.hpp/.cpp     # 控制器（輸出瞄準角）
│   ├── BuffDetector.hpp/.cpp       # 視覺識別（R標+靶心）
│   ├── RosTools.hpp                # 簡單 ROS 工具
│   └── Timer.hpp                   # 計時器（防連打同一葉片）
```

---

## 核心文件詳解

### `main.cpp` — `Application` 類

**啟動流程**：
```
main()
├── rclcpp::init()
├── app.Init("/path/config.json")    // 讀取配置，初始化 BuffDetector 和 BuffCalculator
└── app.Run(argc, argv)              // 主循環
    ├── GenSubs()                    // 訂閱 /ly/ra/enable、/ly/aa/enable、/ly/ra/angle_image、/ly/bullet/speed
    └── while(rclcpp::ok()):
        ├── rclcpp::spin_some()
        ├── if(aa_enable) continue   // 普通瞄準開啟時跳過打符
        ├── if(!Enable) sleep(5s)    // 打符未啟用時等待
        ├── buff_detector().buffDetect(image)   // 識別能量機關
        ├── buff_calculator().calculate(...)    // 計算打擊角度
        └── PubData() → /ly/buff/target
```

**優先級控制**：如果 `aa_enable=true`（普通瞄準開啟），打符被禁用（越級保護）。

#### 訂閱的 Topics

| Topic | 說明 |
|-------|------|
| `/ly/ra/enable` | 打符使能開關（由 `behavior_tree` 控制） |
| `/ly/ra/angle_image` | 帶雲台角的圖像（`detector` 發布的 `AngleImage.msg`） |
| `/ly/aa/enable` | 普通瞄準使能（如果普通瞄準開著則跳過打符） |

#### 發布的 Topics

| Topic | 消息類型 | 說明 |
|-------|----------|------|
| `/ly/buff/target` | `Target.msg` | 打符瞄準角度（`status=true` 時 `behavior_tree` 才會使用） |

---

### `module/BuffDetector.hpp/.cpp`

**`BuffDetector`** — 能量機關視覺識別器

```cpp
bool buffDetect(cv::Mat image);  // 識別成功返回 true
std::vector<cv::Point2f> getCameraPoints();  // 返回 R標+靶心的5個像素點
```

使用基於顏色的圖像分割 + 幾何特徵（R標的圓形特徵 + 靶心形狀）識別能量機關。模型路徑從 `config.json` 的 `buff.buff_model_path` 讀取。

---

### `module/BuffCalculator.hpp/.cpp`（最大的文件，24KB）

**`BuffCalculator`** — 核心算法（位置解算 + 旋轉預測 + 彈道計算）

```cpp
bool calculate(BuffFrame& frame, std::vector<cv::Point2f> camera_points, double bullet_speed);
float get_predictPitch();  // 獲取最終的 pitch（含彈道補償）
float get_predictYaw();    // 獲取最終的 yaw
bool is_shift;             // 是否剛換葉片（觸發計時器重置）
```

**`BuffFrame`**：封裝一幀數據（圖像、時間戳、pitch、yaw、roll）：
```cpp
buff_frame.set(image, steady_clock::now(), pitch, yaw, roll);
```

**計算流程**：
1. 解算 R 標和靶心的3D位置（PnP）
2. 估計當前葉片的旋轉角
3. 預測旋轉速度（小符恆速/大符正弦加速）
4. 計算提前量（飛行時間 × 角速度）
5. 加入彈道補償（重力）
6. 輸出最終 yaw/pitch

---

### `module/Timer.hpp`

**`Timer`** — 防連打計時器

```cpp
Timer timer{MAX_COUNT};  // MAX_COUNT=25 幀
bool call();     // 每幀調用，到達間隔後返回 true（可以開火）
void reset2();   // 換葉片時重置計時器（is_shift=true 時調用）
```

防止在同一片葉片上連續發射多次（要等一定幀數後才允許再次開火）。

---

### `module/BasicTypes.hpp`

打符相關的自定義類型（獨立於 `auto_aim_common`）：
- `BuffFrame`：一幀的圖像+時間+雲台角數據
- `param::Param`：JSON 配置讀取包裝

---

## RA_MultiThreadVariables — 線程安全數據共享

```cpp
struct RA_MultiThreadVariables {
    bool Enable;                        // 打符使能
    cv::Mat Image;                      // 最新圖像
    bool buffHitterShoot;               // 是否開火
    bool buff_follow;                   // 是否跟蹤符
    gimbal_driver::msg::GimbalAngles GimbalAngles;  // 當前雲台角
};
```

通過 `MultiCallback<RA_MultiThreadVariables>` 進行線程安全的數據更新和複製（`GetCopy()` 獲取快照）。

---

## 數據流

```
/ly/ra/angle_image (AngleImage.msg: image + yaw + pitch)
         │
         ▼
RA_MultiThreadVariables (atomic copy)
         │
         ▼
Application::Run() 主循環
├── BuffDetector::buffDetect()      ─► 視覺識別 R標+靶心
├── BuffCalculator::calculate()     ─► 預測旋轉 + 計算角度
└── Timer::call()
         │
         ▼
/ly/buff/target (Target.msg: yaw, pitch, status)  → behavior_tree
```

---

## 修改注意事項

- **配置文件路徑**：`app.Init("/home/hustlyrm/workspace/src/buff_hitter/config/config.json")` 是**硬編碼路徑**，需要改為相對路徑或使用 `ament_index_cpp::get_package_share_directory()`
- **`aa_enable` 回調**：接收的是 `const std_msgs::msg::Bool&`（沒有 `->` 箭頭），這是 ROS1 的值傳遞語法，ROS2 建議使用 `ConstSharedPtr`，但目前這樣也可以工作
- **`gimbal_driver::msg::GimbalAngles`** 字段：代碼中使用 `.pitch` 和 `.yaw`（小寫），確認與 `gimbal_driver/msg/GimbalAngles.msg` 的字段名一致
- **彈速**：当前已订阅 `/ly/bullet/speed`，并支持 `dynamic_bullet_speed_enable/min_bullet_speed/bullet_speed_alpha/default_bullet_speed` 参数
