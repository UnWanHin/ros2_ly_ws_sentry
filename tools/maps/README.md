# Map Tools (`tools/maps`)

這裡放的是「新地圖點位」工具鏈，目的是把官方 STEP 場地模型落地到你現在的 BT/導航流程。

## 為什麼要這套工具

- 你現在的決策主要是發 `/ly/navi/goal`（ID），不是直接發 XY。
- 同一個點位 ID，最終位置由導航側映射決定。
- 新地圖時，最容易出錯的是「BT 座標系」和「導航座標系」不一致。

## 目錄

- `src/`：原始 STEP 檔（你已放 `RMUC2026_V1.2.0.step`）
- `step2png.py`：一鍵把 STEP 轉俯視 PNG（獨立腳本）
- `mappointer.py`：一鍵互動入口（選 STEP、選既有/新建 map、打開標點頁）
- `scripts/step_inspect.py`：讀 STEP 基本資訊（單位、bbox）
- `scripts/map_plugin_cli.py`：管理點位插件 JSON（初始化/驗證/輸出 Area.hpp 片段）
- `scripts/solve_affine.py`：用對照點解 2D 仿射映射（解決「上位機點位 ≠ 導航實際」）
- `web/map_marker.html`：標點工具（載入底圖、紅藍點位點選、匯出 JSON）

## STEP 轉底圖（獨立）

```bash
python3 tools/maps/step2png.py
```

這支只做一件事：`STEP -> PNG`，不會做 map init、不會開標點頁。
預設走 CAD 引擎（`--engine cad`），會輸出等待進度；大型 STEP 會比較久。
若要把場外留白/外框裁掉，再映射成 28m×15m 視野，可加：
`python3 tools/maps/step2png.py --engine cad --crop-field --field-width 28000 --field-height 15000`

## 最簡入口（推薦）

```bash
python3 tools/maps/mappointer.py --image-file tools/maps/basemaps/RMUC2026_V1.2.0_topview.png
```

它會做：

1. 讓你選 `src/` 下的 STEP
2. 讓你選「既有 map 插件」或「新建 map 插件」
3. 輸出可直接打開的標點頁 URL（可選 `--open` 才自動開瀏覽器）

在頁面內可直接用「從 JSON 檔載入」選你剛選的插件檔。

## 建議流程（新地圖）

1. **先檢查 STEP**
   - `python3 tools/maps/scripts/step_inspect.py tools/maps/src/RMUC2026_V1.2.0.step`
2. **把 STEP 做成俯視底圖**
   - 先試：`python3 tools/maps/step2png.py --engine cad`
   - 若想只轉部分 root（大型組件常用）：`python3 tools/maps/step2png.py --engine cad --roots 1`
   - 若你本機 CAD 匯出更快，也可直接用 FreeCAD/SolidWorks/Fusion 匯出頂視圖 PNG 再進標點流程。
3. **標點**
   - 開 `tools/maps/web/map_marker.html`（瀏覽器）。
   - 上傳底圖 PNG，按點位 ID 填紅/藍座標，匯出 `map_plugin.json`。
4. **驗證/產出 C++ 片段**
   - `python3 tools/maps/scripts/map_plugin_cli.py validate --input tools/maps/map_plugin.json`
   - `python3 tools/maps/scripts/map_plugin_cli.py emit-area --input tools/maps/map_plugin.json > /tmp/new_area_snippet.hpp`
5. **若導航側有自己的座標系，先做映射**
   - 準備 3+ 對同名點位（src=BT, dst=導航）。
   - `python3 tools/maps/scripts/solve_affine.py --input tools/maps/control_pairs.json`
   - 把矩陣套回點表後再給導航側。

## 你最常會用的兩個檔

- BT 點位座標定義（紅藍雙座標）：`src/behavior_tree/module/Area.hpp`
- 點位 ID 定義（0..18）：`src/behavior_tree/module/BasicTypes.hpp`
