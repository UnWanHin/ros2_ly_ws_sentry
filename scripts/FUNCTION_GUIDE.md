# Scripts Function Guide

这份文档只讲一件事：

- 在 `scripts/` 下面，常见功能该用哪个入口
- 每个入口适合什么场景
- 最少该怎么跑

如果你只想快速上手，不想先翻完整文档，优先看这份。

---

## 使用前提

先保证工作区已经编过：

```bash
cd ~/ros2_ly_ws_sentary
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

说明：

- 大多数脚本会自己补 `source`，但前提还是工作区里要先有 `install/setup.bash`
- 如果你刚改完代码，建议先重新 `colcon build`

---

## 入口总览

你平时只需要记 3 个顶层入口：

```bash
./scripts/start.sh
./scripts/debug.sh
./scripts/selfcheck.sh
```

分工：

- `start.sh`：正式整链路、展示模式
- `debug.sh`：姿态测试、自瞄联调、导航点位测试、标定
- `selfcheck.sh`：离车/上车自检

---

## 脚本分层

如果你想知道“应该改哪一层”，先记住这张图：

```text
scripts/
├── start.sh / debug.sh / selfcheck.sh
│   └── 给人用的总入口
├── start/ debug/ selfcheck/
│   └── 分类入口，名字稳定，适合记命令
├── launch/
│   └── 真正起 ros2 launch / ros2 run 的实现层
├── feature_test/
│   └── 单项功能测试、standalone 模式
├── tools/
│   └── 拟合、日志过滤等工具
└── config/
    └── 常用 YAML 配置
```

判断原则：

- 想“记命令”：看 `start/`、`debug/`、`selfcheck/`
- 想“改启动参数默认值”：看 `launch/`
- 想“做单项功能测试”：看 `feature_test/`
- 想“改比赛配置”：看 `config/`

---

## 正式比赛整链路

### 1. 分区赛 / 联盟赛主入口

有门控：

```bash
./scripts/start.sh gated --mode regional
./scripts/start.sh gated --mode league
```

无门控：

```bash
./scripts/start.sh nogate --mode regional
./scripts/start.sh nogate --mode league
```

适合：

- 正式主链路启动
- 看完整 `gimbal_driver + detector + tracker_solver + predictor + behavior_tree`

实际脚本：

- `scripts/start/sentry_all.sh`
- `scripts/start/sentry_all_nogate.sh`

---

## 姿态相关

### 2. 姿态展示模式

```bash
./scripts/start.sh showcase
```

适合：

- 展示姿态系统
- 展示巡逻
- 演示 `behavior_tree` 控制下的整车姿态链路

特点：

- 走 `showcase` 配置
- 默认是展示模式，不是正式比赛主入口

### 3. 姿态 topic 直接测试

最常用：

```bash
./scripts/debug.sh posture-test both
```

只发姿态命令：

```bash
./scripts/debug.sh posture-test tx
```

只看下位机姿态回读：

```bash
./scripts/debug.sh posture-test rx
```

只看上位机发出的姿态命令：

```bash
./scripts/debug.sh posture-test echo
```

这个脚本默认会循环发：

- `1 = Attack`
- `2 = Defense`
- `3 = Move`

关键参数：

```bash
./scripts/debug.sh posture-test both --interval 5
./scripts/debug.sh posture-test tx --no-launch-gimbal
./scripts/debug.sh posture-test tx --use-virtual-device true
```

适合：

- 单独验证 `/ly/control/posture`
- 单独验证 `/ly/gimbal/posture`
- 联调下位机姿态回读

---

## 导航相关

### 4. behavior_tree-only 导航调试

```bash
./scripts/debug.sh navi-debug
```

保留门控：

```bash
./scripts/debug.sh navi-debug --with-gate
```

适合：

- 让 `behavior_tree` 自己发导航点
- 调 `NaviDebug` 配置
- 验证 `behavior_tree -> /ly/navi/goal`

### 5. 手动发 `/ly/navi/goal`

```bash
./scripts/debug.sh navi-goal-cli
```

适合：

- 手工输入点位号
- 单独查导航侧是否收到目标

### 6. 按 JSON 巡逻点自动发 `/ly/navi/goal`

```bash
./scripts/debug.sh navi_goal
```

指定计划名：

```bash
./scripts/debug.sh navi_goal --plan test_site_sequence
```

蓝方映射：

```bash
./scripts/debug.sh navi_goal --team blue
```

适合：

- 不走 BT，单独验证 `/ly/navi/goal`
- 临时点位调试
- 快速跑一组巡逻点

对应计划文件：

- `src/behavior_tree/Scripts/ConfigJson/navi_debug_points.json`

### 7. 直接发 `/ly/control/angles`

```bash
./scripts/debug.sh control-angles-test once --yaw 10 --pitch 2
```

循环发固定角：

```bash
./scripts/debug.sh control-angles-test loop --yaw -20 --pitch 5 --interval 1
```

只回显 topic：

```bash
./scripts/debug.sh control-angles-test echo
```

适合：

- 绕过 `predictor/behavior_tree`
- 直接验证 `gimbal_driver` 是否能收角度
- 查下位机角度控制链有没有问题

说明：

- 这个脚本默认可自动起 `gimbal_driver`
- 用的是 `/ly/control/angles`

---

## 直接控制类

### 8. 直接发 rotate 档位

```bash
./scripts/debug.sh rotate_level both
```

只发不看回读：

```bash
./scripts/debug.sh rotate_level tx
```

适合：

- 单独验证 `/ly/control/firecode` 的 Rotate 位
- 查小陀螺档位和下位机回传是否一致

说明：

- 默认循环发 1 -> 2 -> 3
- 对应 firecode：
  - `1 -> 64`
  - `2 -> 128`
  - `3 -> 192`

---

## 自瞄相关

### 9. 比赛风格自瞄主链

```bash
./scripts/debug.sh armor_test
```

也可以切模式：

```bash
./scripts/debug.sh armor_test --mode league
./scripts/debug.sh armor_test --mode regional
```

适合：

- 快速拉起比赛风格 autoaim 链路
- 看 `predictor + behavior_tree` 主链联调

### 10. 自瞄入口简化说明

`autoaim-test` / `autoaim-debug` 入口已下线，统一改用：

```bash
./scripts/debug.sh armor_test
```

如需测单项功能（底盘、导航、前哨、打符等），走 standalone 菜单：

### 11. standalone 菜单

```bash
./scripts/debug.sh standalone
```

适合：

- 想从菜单里挑单项模式
- 不想直接记 `feature_test/standalone/modes/*.sh`

里面常见会包含：

- armor
- buff
- outpost
- navi patrol
- chassis spin

---

## 旋转 / 底盘相关

### 13. Rotate_level（小陀螺）

```bash
./scripts/debug.sh rotate_level both
```

适合：

- 基础 Rotate 链路测试
- 查 rotate 指令与回读是否一致

### 14. Move_Rotate（旋转 + 正弦平移）

```bash
./scripts/debug.sh move_rotate
```

适合：

- 做更连续、更接近演示状态的底盘动作
- 查周期性运动下的链路稳定性

说明：

- 走 `feature_test/standalone/modes/chassis_spin_sine_translate_mode.sh`
- 更偏功能测试，不是正式比赛入口

### 16. 弹道/锁敌日志过滤

```bash
./scripts/debug.sh ballistic-log
```

适合：

- 只盯弹道异常
- 只盯锁敌相关日志
- 避免在一堆总日志里找问题

本质上是：

- `scripts/tools/monitor_ballistic_errors.sh`

---

## 标定相关

### 17. 射表标定

```bash
./scripts/debug.sh shooting-table-calib --team red --output screen
```

适合：

- 标定射表
- 人工微调落点

### 18. 打符射表标定

```bash
./scripts/debug.sh buff-shooting-table-calib --calib-mode periodic --csv-strategy latest
```

适合：

- 采集打符开火时刻的预测特征（角度/距离/高度/旋转角）
- 离线拟合 `buff_config` 的静态和周期补偿参数
- 不接管比赛控制接口，只做标定插件采样

常用离线拟合：

```bash
./scripts/launch/buff_shooting_table_calib.sh --fit-static-latest
./scripts/launch/buff_shooting_table_calib.sh --fit-periodic-latest
```

---

## 自检相关

### 19. 开发机自检

```bash
./scripts/selfcheck.sh pc
```

不重新编译：

```bash
./scripts/selfcheck.sh pc --no-build
```

### 20. 车上自检

```bash
./scripts/selfcheck.sh robot
```

带频率检查：

```bash
./scripts/selfcheck.sh robot --with-hz
```

### 21. 核心套件自检

```bash
./scripts/selfcheck.sh sentry
```

只跑运行态：

```bash
./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12 --skip-hz
```

适合：

- 出车前检查
- 改完主链后快速确认基础链路还活着

---

## 功能到入口的快速映射

### 如果你想测姿态

用：

```bash
./scripts/debug.sh posture-test both
```

### 如果你想测试 `/ly/navi/goal`

手动发：

```bash
./scripts/debug.sh navi-goal-cli
```

自动巡逻发：

```bash
./scripts/debug.sh navi_goal
```

让 BT 自己发：

```bash
./scripts/debug.sh navi-debug
```

### 如果你想测 `/ly/control/angles`

用：

```bash
./scripts/debug.sh control-angles-test once --yaw 10 --pitch 2
```

### 如果你想测 `/ly/control/firecode` 的 Rotate 位

用：

```bash
./scripts/debug.sh rotate_level both
```

### 如果你想拉完整比赛主链

用：

```bash
./scripts/start.sh gated --mode regional
```

### 如果你想做展示模式

用：

```bash
./scripts/start.sh showcase
```

### 如果你想做比赛风格 autoaim 联调

用：

```bash
./scripts/debug.sh armor_test
```

### 如果你想做离车检查

用：

```bash
./scripts/selfcheck.sh pc
```

---

## 我最建议记住的命令

姿态：

```bash
./scripts/debug.sh posture-test both
```

手动发导航点：

```bash
./scripts/debug.sh navi-goal-cli
```

自动发导航点：

```bash
./scripts/debug.sh navi_goal
```

BT 导航调试：

```bash
./scripts/debug.sh navi-debug
```

比赛主链：

```bash
./scripts/start.sh gated --mode regional
```

展示姿态：

```bash
./scripts/start.sh showcase
```

比赛风格 autoaim：

```bash
./scripts/debug.sh armor_test
```

开发机自检：

```bash
./scripts/selfcheck.sh pc
```

---

## 补充说明

- 这份文档只讲 `scripts/` 怎么用，不展开模块实现细节。
- 如果你要看链路原理，去看：
  - `docs/modules/behavior_tree.md`
  - `docs/modules/predictor.md`
  - `docs/architecture/control_angles_data_flow.md`
  - `docs/architecture/fire_control_flow.md`
