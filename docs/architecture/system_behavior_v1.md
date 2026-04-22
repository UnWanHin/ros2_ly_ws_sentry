# 系统行为说明（v1 归档，2026-04-22 校正）

> 本文件保留 `v1` 名称用于历史索引，但内容已按当前代码路径校正。

## 1. 现在 launch 后会自动打吗

结论取决于是否有“控制端”把目标消息转换成 `/ly/control/*`：

1. 只有感知链（如仅启动 `detector` 相关 launch）: 不会自动打。
2. 启动 `behavior_tree`: 会按决策逻辑下发角度和火控。
3. 不启 `behavior_tree` 但启桥接节点（`mapper_node`/脚本）: 可做测试直连开火。

## 2. 当前真实链路

### 2.1 自瞄

`detector -> tracker_solver -> predictor -> /ly/predictor/target -> behavior_tree -> /ly/control/* -> gimbal_driver -> 下位机`

### 2.2 前哨

`detector -> /ly/outpost/armors -> outpost_hitter -> /ly/outpost/target -> behavior_tree -> /ly/control/* -> gimbal_driver -> 下位机`

### 2.3 打符

`detector(/ly/ra/angle_image) -> buff_hitter -> /ly/buff/target -> behavior_tree -> /ly/control/* -> gimbal_driver -> 下位机`

## 3. 关键接口对照

- `gimbal_driver` 订阅：`/ly/control/angles`、`/ly/control/firecode`、`/ly/control/vel`、`/ly/control/posture`
- `behavior_tree` 订阅：`/ly/predictor/target`、`/ly/outpost/target`、`/ly/buff/target`
- `behavior_tree` 发布：`/ly/control/*`（控制下发）

## 4. 快速自检

```bash
ros2 topic info /ly/predictor/target -v
ros2 topic info /ly/control/angles -v
ros2 topic info /ly/control/firecode -v
```

预期：

1. `/ly/predictor/target`：`predictor` 发布，`behavior_tree` 订阅。
2. `/ly/control/angles`：`behavior_tree`（或测试桥接）发布，`gimbal_driver` 订阅。
3. `/ly/control/firecode`：`behavior_tree`（或测试桥接）发布，`gimbal_driver` 订阅。
