# 輔瞄調參交接指南（接手必讀）

本指南回答三個常見問題：

1. 輔瞄是不是 `detector + buff_hitter + outpost_hitter`？
2. 主要參數是不是都在 `detector` 下？
3. 調參是不是只改 YAML 就可以？

## 1. 先釐清鏈路

- 打車輔瞄（常規 Auto-Aim）主鏈路：  
  `detector -> tracker_solver -> predictor -> behavior_tree -> gimbal_driver`
- 打符鏈路（RA）：  
  `detector -> buff_hitter -> behavior_tree -> gimbal_driver`
- 前哨鏈路（Outpost）：  
  `detector -> outpost_hitter -> behavior_tree -> gimbal_driver`

所以「輔瞄」嚴格說通常是第一條打車鏈路；`buff_hitter/outpost_hitter` 是專項模式，不是常規輔瞄主鏈路。

## 2. 參數來源總覽

### A. 優先調這裡（可熱切換，重啟節點生效）

- 檔案：`src/detector/config/auto_aim_config.yaml`
- 影響模塊：
  - `detector`（相機 SN、曝光增益、模型路徑、視頻源、可視化）
  - `tracker_solver / predictor / outpost_hitter(PoseSolver)`（`solver_config.*`）
  - `predictor`（`controller_config.*` 射表與延時）
- 關鍵區塊：
  - `camera_param.*`
  - `detector_config.*`
  - `solver_config.*`
  - `controller_config.*`
  - `io_config.use_virtual_device`（離車/上車切換）

注意：目前保留了 `点号` 和 `斜杠` 兩套 key 兼容，調參時兩套值要保持一致。

### B. 不在 YAML，需改 JSON

- 檔案：`src/buff_hitter/config/config.json`
- `buff_hitter` 仍以這個 JSON 為基礎配置，但已支持 `buff_config.*` 的 YAML 覆蓋層（由 launch 注入）。
- 建議：常改項先放 `src/buff_hitter/config/buff_config.yaml`；基礎模板保留在 JSON。

### C. 不在配置檔，需改代碼重編譯

- `outpost_hitter`：
  - `muzzle_solver->setBulletSpeed(23.0)` 寫死在代碼。
  - `pitch_setpoint -= 2.0`、`yaw` 門限 `±80` 寫死在代碼。
- `buff_hitter`：
  - `main.cpp` 裡實際解算彈速是常量 `22.9`，不是 JSON 內 `controller.bullet_speed`。

結論：**不是只改 YAML 就夠**。常規輔瞄多數可在 YAML 調，但打符/前哨仍有 JSON 和代碼層調參點。

## 3. 建議調參流程（穩定優先）

1. 先改 `auto_aim_config.yaml`（相機、solver、controller）。
2. 打符模式再改 `buff_hitter/config/config.json`。
3. 若涉及前哨彈速/角度偏置，再做代碼改動並 `colcon build --packages-select outpost_hitter buff_hitter`。
4. 每次改動後執行：
   - 離車：`./scripts/selfcheck.sh pc`
   - 上車：`./scripts/selfcheck.sh robot`

## 4. 交接時必記錄

- 改了哪個檔案、哪個鍵、改前改後數值。
- 改動適用場景（離車視頻 / 真機 / 比賽）。
- 是否需要重編譯（YAML/JSON 通常不用；代碼一定要）。
