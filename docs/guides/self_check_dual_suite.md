# 双套自检指南（电脑端 + 车端）

本指南对应两套脚本：

- 电脑端（离车）：`scripts/self_check_pc.sh`
- 车端（上车）：`scripts/self_check_robot.sh`

目标是把“能编译/能启动”和“实机链路正常”分开检查，减少上车排障时间。

## 1. 电脑端离车自检

用途：开发机先做构建、静态契约和 launch 语法检查，不依赖硬件。

```bash
cd ~/ros2_ly_ws_sentary
./scripts/self_check_pc.sh
```

常用参数：

```bash
# 不重建，只跑静态检查
./scripts/self_check_pc.sh --no-build

# 指定包并执行 colcon test
./scripts/self_check_pc.sh --packages "behavior_tree outpost_hitter predictor" --test
```

## 2. 车端上车自检

用途：在车载机验证串口/网络可达和整链路运行时 topic 契约。

```bash
cd ~/ros2_ly_ws_sentary
./scripts/self_check_robot.sh
```

常用参数：

```bash
# 含频率采样（更严格）
./scripts/self_check_robot.sh --with-hz

# 传给 sentry_all.launch.py 的参数
./scripts/self_check_robot.sh -- --config_file:=/abs/path/auto_aim_config.yaml
```

## 3. 基础套件说明

两套脚本都复用主套件：

```bash
./scripts/self_check_sentry.sh
```

新增模式：

- `--static-only`：只检查文件/配置/BT XML（离车）
- `--runtime-only`：只检查 node/topic/hz（上车）

你常用的运行时命令（离车先验证 ROS 图）：

```bash
./scripts/self_check_sentry.sh --runtime-only --launch --wait 12 --skip-hz
```

完整频率版：

```bash
./scripts/self_check_sentry.sh --runtime-only --launch --wait 12
```

## 4. 判定标准

- `FAIL=0` 才算通过。
- `WARN>0` 可继续，但要逐条确认是否为当前场景预期（如未接网线导致 192.168.12.1 不通）。
