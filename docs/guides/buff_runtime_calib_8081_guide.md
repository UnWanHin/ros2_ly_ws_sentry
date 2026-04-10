# 打符运行、标定与 8081 可视化指南（ROS2）

## 1. 适用范围

本文对应当前 ROS2 主链路：

- 打符主节点：`buff_hitter`
- 打符标定插件：`buff_shooting_table_calib`
- 8081 可视化：`detector` + `VideoStreamer`
- 配置分层：`base + module + override`

不再依赖 `change_buff_infantry` 目录运行。

---

## 2. 配置层级（先看这个）

当前启动默认是三层参数，后者覆盖前者：

1. `config/base_config.yaml`
2. 模块配置（例如 `src/detector/config/detector_config.yaml`、`src/buff_hitter/config/buff_config.yaml`）
3. `config/override_config.yaml`

因此你改参数时，优先改模块配置；临时实验建议只写到 `override_config.yaml` 或命令行覆盖，避免污染基线。

---

## 3. 打符主链路怎么用

## 3.1 一键启动整链路

在工作区根目录执行：

```bash
./scripts/launch/start_sentry_all.sh
```

离线回放模式（无真机）：

```bash
./scripts/launch/start_sentry_all.sh --offline
```

## 3.2 打符关键参数（`src/buff_hitter/config/buff_config.yaml`）

- `buff_config.default_mode`
  - `0`：自动判大小符（推荐）
  - `1`：强制小符
  - `2`：强制大符
- `buff_config.mode_switch_topic_enable`
  - `true`：允许话题 `/ly/ra/mode` 覆盖模式
- `buff_config.two_target_enable`
  - `true`：双目标切换流程开启
- `buff_config.dynamic_bullet_speed_enable`
  - `true`：用上发弹速参与解算（下发仍可固定）
- `buff_config.static_shoot_table_adjust_enable`
  - 静态补偿开关
- `buff_config.periodic_shoot_table_adjust_enable`
  - 大符周期补偿开关

## 3.3 运行时切换模式（可选）

如果 `mode_switch_topic_enable=true`，可临时切模式：

```bash
# 强制小符
ros2 topic pub -1 /ly/ra/mode std_msgs/msg/UInt8 "{data: 1}"

# 强制大符
ros2 topic pub -1 /ly/ra/mode std_msgs/msg/UInt8 "{data: 2}"
```

---

## 4. 8081 可视化新设计怎么开

可视化流地址：

```text
http://127.0.0.1:8081/stream
```

配置文件：`src/detector/config/detector_config.yaml`

核心开关：

- `detector_config.web_show`：总开关（8081）
- `detector_config.overlay_predictor_center`：预测中心点
- `detector_config.overlay_predictor_full`：四装甲点/状态点
- `detector_config.overlay_predictor_text`：预测数字叠加（dyaw/st + 左侧状态文本）
- `detector_config.overlay_predictor_timeout_ms`：预测叠加超时
- `detector_config.overlay_buff_text`：打符数字叠加（右侧）
- `detector_config.overlay_buff_show_position`：是否显示 buff 目标 xyz
- `detector_config.overlay_buff_timeout_ms`：buff 叠加超时

建议调试配置：

```yaml
"detector_config.web_show": true
"detector_config.overlay_predictor_center": true
"detector_config.overlay_predictor_full": true
"detector_config.overlay_predictor_text": true
"detector_config.overlay_buff_text": true
"detector_config.overlay_buff_show_position": false
```

如果画面只有框没有预测/打符数字，先检查：

1. `predictor` 是否在发 `/ly/predictor/vis`、`/ly/predictor/debug`
2. `buff_hitter` 是否在发 `/ly/buff/debug`
3. overlay 开关是否被 `override_config.yaml` 或命令行覆盖回 `false`

---

## 5. 打符标定流程（静态 + 周期）

## 5.1 采样节点启动

```bash
./scripts/launch/buff_shooting_table_calib.sh
```

常用参数：

```bash
./scripts/launch/buff_shooting_table_calib.sh \
  --calib-mode static \
  --csv-strategy new \
  --record-dir ~/workspace/record
```

说明：

- `calib_mode=static`：偏静态补偿样本
- `calib_mode=periodic`：偏大符周期补偿样本
- `calib_mode=all`：全收，后处理过滤

采样触发默认基于 `/ly/gimbal/firecode` 上升沿（`sample_on_rising_edge=true`）。

## 5.2 拟合（离线）

静态补偿拟合（最新一份）：

```bash
./scripts/launch/buff_shooting_table_calib.sh --fit-static-latest
```

周期补偿拟合（最新一份）：

```bash
./scripts/launch/buff_shooting_table_calib.sh --fit-periodic-latest
```

拟合写回 `buff_config.yaml`（会自动备份）：

```bash
./scripts/launch/buff_shooting_table_calib.sh \
  --fit-static-latest \
  --write-config src/buff_hitter/config/buff_config.yaml

./scripts/launch/buff_shooting_table_calib.sh \
  --fit-periodic-latest \
  --write-config src/buff_hitter/config/buff_config.yaml
```

## 5.3 生效方式

写回后确认开关：

- `buff_config.static_shoot_table_adjust_enable: true`
- `buff_config.periodic_shoot_table_adjust_enable: true`

然后重启链路即可。

---

## 6. 常见问题

## 6.1 8081 没有预测/打符点

- `web_show` 未开
- overlay 项未开
- 节点没在发 debug topic

## 6.2 标定一直不落样本

- `require_valid_debug=true` 时，`/ly/buff/debug.status` 长期为 `false`
- `firecode` 没有上升沿
- 采样间隔过长（`min_sample_interval_sec`）

## 6.3 拟合后效果不稳定

- 静态/周期模型混用范围不清（先静态后周期）
- 样本覆盖不足（距离/高度/角度）
- 回写到了错误配置层（被 override 覆盖）

