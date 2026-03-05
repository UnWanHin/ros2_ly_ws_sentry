# detector — 視覺檢測節點

## 概述

`detector` 是視覺感知的核心節點，負責從相機讀取圖像，進行裝甲板和車輛的實時檢測，輸出帶有3D位置信息的裝甲板列表。

**主要功能**：
1. 從工業相機（Galaxy大恆）或視頻文件獲取圖像
2. 用 YOLO 模型檢測裝甲板和車輛
3. 對裝甲板進行顏色過濾（只保留敵方）
4. 用 PnP 算法解算裝甲板的3D位置
5. 發布結果給 `tracker_solver` 或 `outpost_hitter`

---

## 目錄結構

```
detector/
├── CMakeLists.txt
├── package.xml
├── detector_node.cpp           # 主節點（多線程圖像循環）
├── config/
│   └── auto_aim_config.yaml    # 統一配置文件（全鏈路共用）
├── launch/
│   ├── auto_aim.launch.py      # 自瞄檢測鏈路
│   ├── outpost.launch.py       # 前哨鏈路
│   ├── buff.launch.py          # 打符鏈路
│   └── old_auto_aim.launch     # 歷史兼容入口（僅調試對照）
├── include/
│   ├── armor_detector/         # 裝甲板檢測器頭文件
│   │   ├── armor_detector.hpp
│   │   ├── armor_filter.hpp
│   │   ├── armor_refinder.hpp
│   │   ├── corrected_detector.hpp
│   │   ├── number_classifier.hpp
│   │   └── pnp_solver.hpp
│   ├── car_detector/           # 車輛檢測器頭文件
│   │   ├── car_finder.hpp
│   │   └── yolo_detector.hpp
│   ├── camera/                 # 相機接口
│   │   ├── Camera.hpp
│   │   ├── DxImageProc.h       # 大恆相機圖像處理API
│   │   └── GxAPI.h             # 大恆相機GxAPI
│   └── car_and_armor_detector.hpp  # 裝甲+車輛聯合檢測器接口
├── src/                        # 實現文件
│   ├── armor_detector.cpp
│   ├── armor_filter.cpp
│   ├── armor_refinder.cpp
│   ├── number_classifier.cpp
│   ├── pnp_solver.cpp
│   └── yolo_detector.cpp
├── module/
│   └── Camera.hpp              # 相機封裝（工業相機+視頻+ROS bag）
├── script/
│   ├── fire_flip_test.py       # 火控翻轉壓測（可選自啟 detector）
│   └── mapper_node.py          # Target->雲台角映射與測試火控
└── Extras/                     # 模型文件
    ├── armor_detector_model.xml/bin    # 裝甲板 YOLO 模型（OpenVINO格式）
    ├── car_detector_model.xml/bin      # 車輛 YOLO 模型
    ├── classifier.xml/bin/svm          # 數字分類器模型
    └── *.zip                           # 備份壓縮包
```

---

## 啟動入口（ROS 2）

推薦按模式啟動：

```bash
ros2 launch detector auto_aim.launch.py
ros2 launch detector outpost.launch.py
ros2 launch detector buff.launch.py
```

保留舊入口（僅調試對照，不建議日常使用）：

```bash
ros2 launch detector old_auto_aim.launch
```

---

## 測試腳本（script/）

所有腳本均建議在工作區根目錄執行，並先完成：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

`fire_flip_test.py`：持續翻轉 `/ly/control/firecode`，用於硬件火控鏈路測試。

```bash
python3 src/detector/script/fire_flip_test.py --fire-hz 8.0
python3 src/detector/script/fire_flip_test.py --start-detector true --params-file src/detector/config/auto_aim_config.yaml
```

`mapper_node.py`：訂閱 `/ly/predictor/target`，轉發到 `/ly/control/angles`，並可按目標狀態翻轉火控。

```bash
python3 src/detector/script/mapper_node.py --target-id 6 --enable-fire true --auto-fire true
```

---

## 核心文件詳解

### `detector_node.cpp` — 主節點

整個節點以**多線程**方式運行，避免圖像處理阻塞 ROS 回調。

#### 全局狀態變量

| 變量 | 類型 | 說明 |
|------|------|------|
| `myTeamRed` | `atomic_bool` | 我方是否紅隊（由 `/ly/me/is_team_red` 更新） |
| `aa_enable` | `atomic_bool` | 普通瞄準是否啟用（默認 true） |
| `ra_enable` | `atomic_bool` | 打符模式是否啟用（默認 false） |
| `outpost_enable` | `atomic_bool` | 前哨站瞄準是否啟用（默認 false） |
| `atomic_target` | `atomic<ArmorType>` | 當前打擊目標類型（由 `behavior_tree` 通過 `/ly/bt/target` 指定） |
| `gimbal_angles_yaw/pitch` | `atomic<float>` | 最新的雲台角（由 `/ly/gimbal/angles` 更新） |

#### 訂閱的 Topics

| Topic | 回調函數 | 作用 |
|-------|----------|------|
| `/ly/me/is_team_red` | `my_team_callback` | 更新 `myTeamRed`，控制過濾器保留哪種顏色的裝甲板 |
| `/ly/gimbal/angles` | `gimbal_callback` | 更新當前雲台角，綁定到每幀圖像 |
| `/ly/bt/target` | `get_target_callback` | 更新目標裝甲板類型，傳給 `ArmorFilter` |
| `/ly/aa/enable` | `aa_enable_callback` | 切換普通瞄準模式 |
| `/ly/compressed/image` | `image_callback` | 僅在 `use_ros_bag=true` 時使用，從 bag 文件讀取圖像 |

#### 發布的 Topics

| Topic | 條件 | 說明 |
|-------|------|------|
| `/ly/detector/armors` | `aa_enable=true` | 普通瞄準裝甲板列表（→`tracker_solver`） |
| `/ly/outpost/armors` | `outpost_enable=true` | 前哨站瞄準裝甲板列表（→`outpost_hitter`） |
| `/ly/compressed/image` | `pub_image=true` | JPEG壓縮圖像（→調試/錄像） |
| `/ly/ra/angle_image` | `ra_enable=true` | 帶雲台角的圖像（→`buff_hitter`） |

#### `ImageLoop()` — 三線程結構

```
ImageLoop()
├── std::jthread imagepub_thread    // 發布壓縮圖像（JPEG, 80Hz）
│   └── ImageQueue → /ly/compressed/image
│
├── std::jthread ra_imagepub_thread // 發布帶角度圖像給打符（100Hz）
│   └── angle_image_stack → /ly/ra/angle_image
│
└── std::jthread detect_thread      // 主檢測循環（78Hz）
    ├── Cam.GetImage()              // 從相機/視頻/bag獲取圖像
    ├── spin_some()                 // 處理 ROS 回調（更新雲台角等）
    ├── carAndArmorDetector.Detect()// 聯合檢測
    ├── filter.Filter()             // 顏色+目標過濾
    ├── finder.ReFindAndSolveAll()  // PnP解算
    ├── carFinder.FindCar()         // 車輛匹配
    └── Publisher → /ly/detector/armors 或 /ly/outpost/armors
```

> **重要設計**：`imagepub_thread` 和 `detect_thread` 用 `ImageQueue`（帶鎖的隊列）解耦，避免圖像發布阻塞檢測。打符的圖像用 `boost::lockfree::stack`（無鎖棧），只保留最新幀。

---

### `include/car_and_armor_detector.hpp` — 聯合檢測器

封裝了裝甲板檢測器和車輛檢測器：

```cpp
struct CarAndArmorDetector {
    ArmorDetector armorDetector;   // 包含 Corrector.Classifier (數字分類) 和 Detector (YOLO)
    YoloDetector carDetector;      // 車輛 YOLO 檢測器
    bool Detect(cv::Mat& image, std::vector<ArmorObject>& armors, std::vector<CarDetection>& cars);
};
```

---

### `src/armor_detector.cpp` + `include/armor_detector/armor_detector.hpp`

裝甲板 YOLO 檢測器的封裝：
- 輸入：BGR 圖像
- 輸出：`std::vector<ArmorObject>`（包含角點、類型等）
- 使用 OpenVINO 推理引擎，模型文件在 `Extras/armor_detector_model.xml/bin`

---

### `src/armor_filter.cpp` + `include/armor_detector/armor_filter.hpp`

**`ArmorFilter`** — 對 YOLO 輸出的裝甲板列表進行過濾：

| 過濾條件 | 說明 |
|----------|------|
| 顏色過濾 | 根據 `is_team_red` 只保留敵方顏色的裝甲板 |
| 目標類型過濾 | 根據 `atomic_target` 優先選擇指定類型（behavior_tree 設置） |
| 距離圖中心 | 保留最近圖像中心的裝甲板（`distance_to_image_center`） |

---

### `src/armor_refinder.cpp` + `include/armor_detector/armor_refinder.hpp`

**`ArmorRefinder`** — 對過濾後的裝甲板進行精化和PnP解算dispatch：
- 調用 `PoseSolver` 對每個裝甲板解算3D位置
- 更新 `Armor.msg` 的 `rotation[]` 和 `translation[]` 字段
- 選出最優的目標裝甲板 `target_armor`

---

### `src/number_classifier.cpp` + `include/armor_detector/number_classifier.hpp`

**`NumberClassifier`** — 裝甲板數字分類器：
- 輸入：裝甲板 ROI 圖像
- 輸出：裝甲板類型（1=英雄、2=工程、3=步兵1、4=步兵2、5=哨兵、7=前哨）
- 使用 SVM 分類器（`Extras/classifier.svm`）或 OpenVINO 模型（`Extras/classifier.xml/bin`）

---

### `src/pnp_solver.cpp` + `include/armor_detector/pnp_solver.hpp`

**`PoseSolver`** — 使用 OpenCV 的 `solvePnP` 求解裝甲板在相機座標系下的3D位置：
- 輸入：4個角點像素座標 + 相機內參（`CameraIntrinsicsParameterPack`）
- 輸出：旋轉向量 + 位移向量
- 結果寫入 `Armor.msg` 的 `rotation[]` 和 `translation[]`

---

### `src/yolo_detector.cpp` + `include/car_detector/yolo_detector.hpp`

**`YoloDetector`** — 車輛 YOLO 檢測器（OpenVINO）：
- 模型：`Extras/car_detector_model.xml/bin`
- 輸入：BGR 圖像
- 輸出：`std::vector<CarDetection>`（BBox + car_id）

---

### `include/car_detector/car_finder.hpp`

**`CarFinder`** — 將 YOLO 車輛檢測結果轉換為 `auto_aim_common/msg/Car` 列表，加入 `Armors.msg.cars[]`

---

### `include/camera/Camera.hpp` + `module/Camera.hpp`

**`Camera`** — 統一的圖像獲取接口，支持三種模式：
1. **大恆工業相機**（默認）：通過 GxAPI 獲取原始圖像
2. **視頻文件**（`use_video=true`）：使用 OpenCV VideoCapture 讀取 mp4
3. **ROS bag**（`use_ros_bag=true`）：通過 `callbackQueue` 接收壓縮圖像回放

---

## 配置文件 `auto_aim_config.yaml`

| 參數路徑 | 類型 | 默認值 | 說明 |
|----------|------|--------|------|
| `detector_config/show` | bool | true | 是否發布壓縮圖像 |
| `detector_config/draw` | bool | true | 是否在圖像上繪製裝甲板 |
| `detector_config/debug_mode` | bool | false | 調試模式（忽略隊伍顏色） |
| `detector_config/debug_team_blue` | bool | false | 調試時強制使用藍隊過濾 |
| `detector_config/save_video` | bool | false | 是否保存視頻 |
| `detector_config/web_show` | bool | true | 是否啟用網頁實時預覽 |
| `detector_config/use_video` | bool | false | 是否使用視頻文件輸入 |
| `detector_config/use_ros_bag` | bool | false | 是否使用 ROS bag 輸入 |
| `detector_config/video_path` | string | "" | 視頻文件路徑 |
| `detector_config/classifier_path` | string | | 數字分類器模型路徑 |
| `detector_config/detector_path` | string | | 裝甲板YOLO模型路徑 |
| `detector_config/car_model_path` | string | | 車輛YOLO模型路徑 |
| `camera_param/ExposureTime` | double | 4000 | 曝光時間（μs） |
| `camera_param/Gain` | double | 12 | 增益 |
| `camera_param/RedBalanceRatio` | double | 1.2266 | R通道白平衡 |
| `camera_param/GreenBalanceRatio` | double | 1.0 | G通道白平衡 |
| `camera_param/BlueBalanceRatio` | double | 1.3711 | B通道白平衡 |

---

## 數據流

```
相機/視頻/bag
    │
    ▼
Cam.GetImage() ──► detect_thread
    │
    ├── gimbal_angles（原子量）  ◄── /ly/gimbal/angles
    │
    ▼
CarAndArmorDetector.Detect()
    ├── YOLO（裝甲板）
    └── YOLO（車輛）
    │
    ▼
ArmorFilter.Filter()  ◄── myTeamRed, atomic_target
    │
    ▼
ArmorRefinder.ReFindAndSolveAll()  ─► PnP解算（rotation, translation）
    │
    ▼
Armors.msg
    ├── /ly/detector/armors  ─► tracker_solver（aa_enable=true時）
    └── /ly/outpost/armors   ─► outpost_hitter（outpost_enable=true時）
```

---

## 修改注意事項（你最近改了的部分）

- `detector_node.cpp` 是你最近修改的文件，用於即時測試瞄準
- 你增加了 `ImageQueue`（帶鎖）替換原來的隊列，並新增了 `angle_image_stack`（無鎖棧）用於打符圖像
- `use_ros_bag` 模式下從 `/ly/compressed/image` 接收圖像，**注意** `image_callback` 只在此模式下被綁定
- 單獨火控鏈路測試統一使用 `script/mapper_node.py` 與 `script/fire_flip_test.py`
