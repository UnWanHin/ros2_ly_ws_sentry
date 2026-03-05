# Scripts Usage Guide

本目录放运行与自检脚本。下面按“用途 -> 命令 -> 结果判定”给出最常用入口。

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
- 会调用 `self_check_sentry.sh --launch` 自动拉起整链路。

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

# 仅运行时链路（你提到的命令）
./scripts/self_check_sentry.sh --runtime-only --launch --wait 12 --skip-hz

# 运行时链路 + 频率
./scripts/self_check_sentry.sh --runtime-only --launch --wait 12
```

## 4. `start_sentry_all.sh`（整链路启动）

用途：
- 直接启动 `sentry_all.launch.py`，常用于手工调试或配合其他脚本。

```bash
./scripts/start_sentry_all.sh
```

## 结果判定

- `FAIL: 0`：通过。
- `WARN > 0`：可继续，但需确认是否为当前场景预期（如未接网线）。
