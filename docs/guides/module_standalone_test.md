# 子模块单独测试指南（ROS 2）

这份指南用于**不启动整套哨兵系统**时，单独拉起某个模块做联调。

## 通用准备

```bash
cd ~/ros2_ly_ws_sentary
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

可先做 launch 语法检查：

```bash
export ROS_LOG_DIR=/tmp/ros2_logs
mkdir -p ${ROS_LOG_DIR}
ros2 launch <pkg> <launch_file> --show-args
```

## 调试模式现状（按当前代码）

### A) 感知链路调试（不控火）

```bash
ros2 launch detector auto_aim.launch.py offline:=true
```

- 启动：`gimbal_driver + detector + tracker_solver + predictor`。
- 结果：只会产出检测/预测数据，不会主动发布 `/ly/control/firecode`。
- `offline:=true` 会强制虚拟串口与视频回放覆盖，无需反复改 YAML。

### B) 火控联调（离车推荐）

```bash
# 终端 1
ros2 launch detector auto_aim.launch.py offline:=true

# 终端 2
ros2 run detector mapper_node \
  --target-priority 6,3,4,5 \
  --target-id 6 \
  --publish-team false \
  --enable-fire true \
  --auto-fire true
```

- `mapper_node` 默认仅接管 `/ly/control/angles`、`/ly/control/firecode` 与 `/ly/bt/target`。
- 队伍颜色默认不由 mapper 发布（`--publish-team false`），仍由 `gimbal_driver` 的裁判链路提供。
- 联调时确保单控制源：不要与 `behavior_tree` 同时写 `/ly/control/angles`、`/ly/control/firecode`。

可直接使用脚本一键拉起（默认 `fire` 模式）：

```bash
./scripts/debug.sh autoaim-debug
```

仅感知链（不控火）：

```bash
./scripts/debug.sh autoaim-debug --mode perception
```

### E) BT 外配置化功能测试（单控制源）

```bash
./scripts/feature_test/run_feature_test.sh
```

- 配置文件：`scripts/feature_test/config/sentry_feature_test.yaml`
- 当前 Phase 1 支持：
  - 云台：`armor`、`scan`
  - 底盘：`velocity`
- 默认拒绝与 `/behavior_tree` 并行控制；若检测到 `/ly/control/*` 多发布者会直接退出。

### C) 决策联调（behavior_tree）

```bash
ros2 launch behavior_tree sentry_all.launch.py
```

- `behavior_tree` 在进入主循环前会等待 `/ly/game/is_start`，未开赛时会停在等待阶段。
- `Application::CheckDebug()` 只覆盖 `AimMode`（例如强制 Buff/Outpost），不负责跳过开赛等待。
- 若离车环境没有稳定裁判数据，优先使用模式 A/B 做调试。

### D) `detector_config.debug_mode` 的真实作用

- 该开关仅影响 detector 的敌我颜色过滤逻辑：
  - `debug_mode=true + debug_team_blue=true`：强制按蓝方视角过滤。
  - `debug_mode=true + debug_team_blue=false`：强制按红方视角过滤。
- 当前默认值建议为：`debug_mode=false`、`debug_team_blue=false`（优先使用裁判队色）。
- 它不是“全系统调试模式”，不会控制 behavior_tree 的开赛等待流程。

## 1) gimbal_driver 单测

推荐入口（Python launch）：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py
```

虚拟串口测试：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py use_virtual_device:=true
```

兼容入口（XML）：

```bash
ros2 launch gimbal_driver gimbal_driver.launch
```

## 2) detector 链路单测

自瞄链路：

```bash
ros2 launch detector auto_aim.launch.py
```

查看可选参数（含 mapper 与隔离话题）：

```bash
ros2 launch detector auto_aim.launch.py --show-args
```

可选：同一条 launch 内启动 mapper（默认隔离到 debug 控制话题，不直接驱动云台）：

```bash
ros2 launch detector auto_aim.launch.py use_mapper:=true
```

若要让 mapper 直接驱动云台（仅限单控制源联调）：

```bash
ros2 launch detector auto_aim.launch.py \
  use_mapper:=true \
  mapper_angles_topic:=/ly/control/angles \
  mapper_firecode_topic:=/ly/control/firecode
```

打符链路：

```bash
ros2 launch detector buff.launch.py
```

纯打符（不启 BT、不跑巡逻；由桥接节点直连 `/ly/buff/target -> /ly/control/*`）：

```bash
./scripts/launch/buff_test.sh
```

前哨链路：

```bash
ros2 launch detector outpost.launch.py
```

保留旧入口（仅调试对照）：

```bash
ros2 launch detector old_auto_aim.launch
```

## 3) behavior_tree 单测

```bash
ros2 launch behavior_tree behavior_tree.launch.py
```

## 4) shooting_table_calib 单测

推荐入口（Python launch）：

```bash
ros2 launch shooting_table_calib shooting_table_calib.launch.py
```

兼容入口（XML）：

```bash
ros2 launch shooting_table_calib shooting_table_calib.launch
```

## 5) detector 脚本单测（火控/映射）

火控翻转压测（持续翻转 `/ly/control/firecode`）：

```bash
python3 src/detector/script/fire_flip_test.py --fire-hz 8.0
```

可选：脚本内同时拉起 detector 节点：

```bash
python3 src/detector/script/fire_flip_test.py --start-detector true --params-file src/detector/config/auto_aim_config.yaml
```

目标映射工具（`/ly/predictor/target` -> `/ly/control/angles`，并发布 `/ly/bt/target` 供目标类型选择）：

```bash
ros2 run detector mapper_node \
  --target-priority 6,3,4,5 \
  --target-id 6 \
  --publish-team false \
  --enable-fire true \
  --auto-fire true
```

说明：
- 默认 `--publish-team false`，队伍颜色由 `gimbal_driver`（裁判系统）发布，避免与 mapper 冲突。
- `--target-priority` 为目标类型优先级（按顺序命中当前可见类型），`--target-id` 为回退目标。
- 若离线无裁判数据需临时注入队伍颜色，可显式开启：

```bash
ros2 run detector mapper_node --publish-team true --red true
```

建议联调时同时观察控制话题发布者，避免多写冲突导致云台抽动：

```bash
ros2 topic info /ly/control/angles -v
ros2 topic info /ly/control/firecode -v
```

若发布者超过 1 个（例如 `behavior_tree` 与 `mapper_node` 同时在写），先停掉其中一个控制源。

## 备注

- 日常整链路仍建议使用：
  `ros2 launch behavior_tree sentry_all.launch.py`
- 单测时若出现参数不生效，优先检查是否正确加载 `detector/config/auto_aim_config.yaml`。
- 若出现“第一次正常、第二次卡住”，先检查是否有残留控制源：
  - `ros2 topic info /ly/control/angles -v`
  - `ros2 topic info /ly/control/firecode -v`
  - `pgrep -af "gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|behavior_tree_node|mapper_node|fire_flip_test"`
- 统一自检入口（推荐）：
  - 离车：`./scripts/selfcheck.sh pc`
  - 上车：`./scripts/selfcheck.sh robot`
