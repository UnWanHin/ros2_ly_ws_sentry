# Scripts Usage Guide

本目录放运行与自检脚本。下面按“用途 -> 命令 -> 结果判定”给出最常用入口。

## 目录分层（已整理）

- `scripts/launch/`：启动类脚本实现（整链路、无门控、auto_aim联调）。
- `scripts/feature_test/standalone/`：单独功能测试（交互入口 + 各模式 + 工具脚本）。
- `scripts/feature_test/`：配置驱动的功能测试框架（Phase 1）。
- `scripts/config/`：测试与比赛配置模板。
- 根目录 `scripts/*.sh`：兼容入口（薄封装，保持旧命令可用）。

## 1. `self_check_pc.sh`（离车自检）

用途：
- 开发机执行，验证构建、静态配置契约、launch 语法。
- 不依赖车端硬件链路。

常用命令：

```bash
# 标准离车自检（会先构建）
./scripts/self_check_pc.sh

# 仅静态检查，不重建
./scripts/self_check_pc.sh --no-build

# 指定包并执行 colcon test
./scripts/self_check_pc.sh --packages "behavior_tree outpost_hitter predictor" --test
```

## 2. `self_check_robot.sh`（上车自检）

用途：
- 车载机执行，检查串口候选设备、权限、网络可达，以及运行时 ROS2 图契约。
- 会调用 `self_check_sentry.sh --runtime-only --launch` 自动拉起整链路。
- 默认只跑运行态（`--runtime-only`），避免重复静态检查。

常用命令：

```bash
# 快速上车自检（默认跳过 hz）
./scripts/self_check_robot.sh

# 严格上车自检（含 hz 采样）
./scripts/self_check_robot.sh --with-hz

# 传递 launch 参数
./scripts/self_check_robot.sh -- --config_file:=/abs/path/auto_aim_config.yaml
```

## 3. `self_check_sentry.sh`（基础自检套件）

用途：
- 核心检查脚本，支持静态/运行时分模式，PC 与 Robot 脚本均复用它。

常用命令：

```bash
# 仅静态（文件、配置、BT XML）
./scripts/self_check_sentry.sh --static-only

# 离线模式运行时链路（自动传 offline:=true）
./scripts/self_check_sentry.sh --runtime-only --launch --offline --wait 12 --skip-hz

# 仅运行时链路（你提到的命令）
./scripts/self_check_sentry.sh --runtime-only --launch --wait 12 --skip-hz

# 运行时链路 + 频率
./scripts/self_check_sentry.sh --runtime-only --launch --wait 12
```

## 4. `start_sentry_all.sh`（整链路启动）

用途：
- 直接启动 `sentry_all.launch.py`，常用于手工调试或配合其他脚本。
- 启动前可用 `1/2` 选择 `competition_profile`（`league/regional`）。
- 默认沿用 launch 的配置文件（`detector/config/auto_aim_config.yaml`）。
- 脚本主要注入模式相关参数（如 `competition_profile` / `bt_config_file` / `offline`），不再强制覆盖 `config_file`。

```bash
./scripts/start_sentry_all.sh

# 离线模式（强制虚拟串口 + 视频回放参数覆盖）
./scripts/start_sentry_all.sh --offline

# 非交互指定模式
./scripts/start_sentry_all.sh --mode 1 --no-prompt     # league
./scripts/start_sentry_all.sh --mode regional --no-prompt

# 如需覆盖配置文件，可显式传入
./scripts/start_sentry_all.sh -- config_file:=/abs/path/auto_aim_config.yaml

# 单独联调时可选（默认都关闭，不影响正式比赛）
./scripts/start_sentry_all.sh -- wait_for_game_start_timeout_sec:=8
./scripts/start_sentry_all.sh -- debug_bypass_is_start:=true
./scripts/start_sentry_all.sh -- league_referee_stale_timeout_ms:=2000
```

二次启动卡住时，优先用清理模式（默认已开启）：

```bash
./scripts/start_sentry_all.sh --cleanup-existing
```

脚本会把 `ROS_LOG_DIR` 默认指向 `/tmp/ros2_logs`，避免 `~/.ros/log` 权限异常导致 launch 直接失败。

若你明确要保留当前正在跑的进程，可禁用清理并在冲突时失败退出：

```bash
./scripts/start_sentry_all.sh --no-cleanup-existing
```

## 4.1 `start_sentry_all_competition.sh`（兼容入口）

用途：
- 兼容旧命令入口，内部仍会调用 `start_sentry_all.sh`。
- 比赛模式选择与 `competition_profile` 对齐逻辑，统一由 `start_sentry_all.sh` 处理。

常用命令：

```bash
# 比赛版启动（会进入 start_sentry_all 的模式选择）
./scripts/start_sentry_all_competition.sh

# 仍可使用 offline 或其它 launch 参数
./scripts/start_sentry_all_competition.sh --offline
./scripts/start_sentry_all_competition.sh -- use_buff:=false use_outpost:=false
```

## 4.2 `start_sentry_all_nogate.sh`（绕过开赛门控）

用途：
- 调试/联调专用，启动时固定注入 `debug_bypass_is_start:=true`。
- 适合在没有裁判系统 `is_start` 的场景下验证上位机与导航等链路。
- 其余参数透传给 `start_sentry_all.sh`。

常用命令：

```bash
# 直接绕过 is_start 门控启动
./scripts/start_sentry_all_nogate.sh

# 联盟赛 + 非交互
./scripts/start_sentry_all_nogate.sh --mode 1 --no-prompt

# 也可继续追加 launch 参数
./scripts/start_sentry_all_nogate.sh -- wait_for_game_start_timeout_sec:=8
```

## 5. `start_autoaim_debug.sh`（auto_aim + mapper 联调）

用途：
- 面向“感知链/火控链”拆分联调，不拉起 behavior_tree。
- 支持三种模式：`perception`（仅感知）、`mapper`（仅 mapper）、`fire`（感知+mapper，默认）。
- 默认 `--offline`，并带残留进程清理、单控制源保护（防止与 behavior_tree 或其他发布者同时写 control 话题）。
- 已加实例锁（`/tmp/sentry_autoaim_debug.lock`），并修复退出时后台 launch 清理。

常用命令：

```bash
# 默认：感知链 + mapper（火控联调）
./scripts/start_autoaim_debug.sh

# 仅感知链（不控火）
./scripts/start_autoaim_debug.sh --mode perception

# 仅 mapper（要求感知链已启动）
./scripts/start_autoaim_debug.sh --mode mapper

# 指定 mapper 目标优先级与回退目标
./scripts/start_autoaim_debug.sh --target-priority "6,3,4,5" --target-id 6

# 在线模式（不传 offline:=true）
./scripts/start_autoaim_debug.sh --online
```

## 5.1 `start_autoaim_test.sh`（一键辅瞄联调）

用途：
- 薄封装入口（已整理到 `feature_test/standalone/modes/armor_mode.sh`），默认等价于 `start_autoaim_debug.sh --mode fire --offline`。
- 适合你只想快速验证辅瞄链路时直接开跑。

常用命令：

```bash
./scripts/start_autoaim_test.sh
./scripts/start_autoaim_test.sh --online
./scripts/start_autoaim_test.sh --target-priority "6,3,4,5" --target-id 6
```

## 5.2 `start_chassis_gyro_test.sh`（一键底盘陀螺链路）

用途：
- 自动拉起最小链路（仅 `gimbal_driver`），然后持续发布 `FireCode.Rotate` 到 `/ly/control/firecode`。
- 用于底盘陀螺/旋转链路联调，不依赖 behavior_tree 与裁判开赛门控。
- 脚本主体已整理到 `feature_test/standalone/modes/chassis_spin_mode.sh`。

常用命令：

```bash
./scripts/start_chassis_gyro_test.sh
./scripts/start_chassis_gyro_test.sh --offline --rotate-level 2
./scripts/start_chassis_gyro_test.sh --mode 1 --hz 30 --wait 6
```

## 5.3 `start_standalone_test.sh`（交互式单入口）

用途：
- 统一单独测试入口，交互选择模式：
  - `1` 打装甲板（Armor）
  - `2` 打大符（Buff）
  - `3` 打前哨（Outpost）
  - `4` 小陀螺（Chassis Spin）
- 统一加了互斥锁（`/tmp/sentry_standalone_test.lock`）和 Ctrl+C 清理，减少重复节点与卡进程。

常用命令：

```bash
# 交互选择模式
./scripts/start_standalone_test.sh

# 非交互直接选模式
./scripts/start_standalone_test.sh --select 1 --online
./scripts/start_standalone_test.sh --select 2 --online --enable-fire true
./scripts/start_standalone_test.sh --select 3 --offline
./scripts/start_standalone_test.sh --select 4 --rotate-level 2
```

目录整理（单独测脚本）：
- `scripts/feature_test/standalone/run_standalone_menu.sh`
- `scripts/feature_test/standalone/modes/armor_mode.sh`
- `scripts/feature_test/standalone/modes/buff_mode.sh`
- `scripts/feature_test/standalone/modes/outpost_mode.sh`
- `scripts/feature_test/standalone/modes/chassis_spin_mode.sh`
- `scripts/feature_test/standalone/tools/control_mode_publisher.py`
- `scripts/feature_test/standalone/tools/target_to_control_bridge.py`

## 5.4 `start_navi_goal_cli.sh`（导航点位序号联调）

用途：
- 手动联调导航链路，交互输入并持续发布：
  - `/ly/navi/goal`（点位序号）
  - `/ly/navi/speed_level`（可选）
- 适合和导航同学联调“序号链路是否打通”。

常用命令：

```bash
# 进入交互模式
./scripts/start_navi_goal_cli.sh

# 可调发布频率
./scripts/start_navi_goal_cli.sh --hz 5
```

交互命令：
- `<id>`：设置并持续发布 goal 序号（0~255）
- `s <speed>`：设置 speed_level
- `<id> <speed>`：同时设置 goal 和 speed
- `p`：查看当前值
- `q`：退出

## 6. `feature_test/run_feature_test.sh`（BT 外功能测试框架，单控制源）

用途：
- 使用单个配置文件启动“云台/底盘”功能测试，运行在 `behavior_tree` 外。
- 默认启用“单控制源保护”：检测到 `/behavior_tree` 或控制话题多发布者时拒绝接管。

默认配置文件：
- `scripts/config/sentry_feature_test.yaml`

常用命令：

```bash
# 按配置运行（默认是 gimbal armor）
./scripts/feature_test/run_feature_test.sh

# 只预览命令，不执行
./scripts/feature_test/run_feature_test.sh --dry-run

# 指定配置文件
./scripts/feature_test/run_feature_test.sh --config ./scripts/config/sentry_feature_test.yaml
```

当前 Phase 1 支持：
- 云台：`armor`、`scan`
- 底盘：`velocity`

## 结果判定

- `FAIL: 0`：通过。
- `WARN > 0`：可继续，但需确认是否为当前场景预期（如未接网线）。
- 若使用 `--launch` 且失败，脚本会自动输出 `Launch Diagnosis`，包含崩溃签名与最近日志尾部。
- 运行前会提示当前配置是否要求“真实相机/真实串口”，避免离车时误用比赛参数。
- 使用 `--offline` 时，会自动传 `offline:=true` 给 launch，避免每次手改 YAML。
- 运行时链路检查已覆盖 posture 合约：`/ly/control/posture` 与 `/ly/gimbal/posture`。
- 若 `ros2 run` 直接报 `Segmentation fault` 且日志里有 `Failed opening file ~/.ros/log/... Permission denied`，先修复日志目录权限或临时设置 `export ROS_LOG_DIR=/tmp/ros2_logs` 再复测。

## 比赛/调试开关位置

统一在：

- `src/detector/config/auto_aim_config.yaml`

重点键位（已在 YAML 内写明“比赛建议/调试建议”注释）：

- `detector_config/use_video`
- `detector_config/debug_mode`
- `detector_config/web_show`
- `detector_config/show`
- `detector_config/draw`
- `detector_config/use_ros_bag`
- `detector_config/save_video`
- `io_config/use_virtual_device`
