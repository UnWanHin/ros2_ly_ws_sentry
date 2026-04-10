# 外部 Topic 边界说明（导航/下位机）

本文档用于明确：哪些链路在本仓库内闭环，哪些链路依赖外部模块（导航上位机/下位机）。

## 1. 仓内闭环链路

- 打车主链：`detector -> tracker_solver -> predictor -> behavior_tree -> gimbal_driver`
- 打符链：`detector -> buff_hitter -> behavior_tree -> gimbal_driver`
- 前哨链：`detector -> outpost_hitter -> behavior_tree -> gimbal_driver`

以上链路的 ROS 节点都在本仓库内可见。

## 2. 外部依赖链路

### 导航接口（通常由外部导航模块消费/生产）

- BT 发布：
  - `/ly/navi/goal`
  - `/ly/navi/goal_pos`
  - `/ly/navi/speed_level`
  - `/ly/navi/target_rel`（追击相对目标点，x/y/z）
- BT 订阅：
  - `/ly/navi/vel`
  - `/ly/navi/lower_head`

说明：本仓库内未包含完整导航执行节点，以上通常由外部导航系统或其他上位机负责。

### 下位机串口接口（由 gimbal_driver 对接）

- 上位控制下发（经 `gimbal_driver` 串口写入）：
  - `/ly/control/angles`
  - `/ly/control/firecode`
  - `/ly/control/vel`
- 下位回传上行（由 `gimbal_driver` 发布）：
  - `/ly/gimbal/*`、`/ly/game/*`、`/ly/team/buff`、`/ly/position/data` 等

## 3. 关键兼容约定

- `behavior_tree` 已恢复速度桥接：`/ly/navi/vel -> /ly/control/vel`。  
  即：导航回传速度先进入 BT，再由 BT 转发给 gimbal_driver。
- 当 `Chase.UseRelativeTargetTopic=true` 时，BT 会发布 `/ly/navi/target_rel`，由导航侧决定速度分配；  
  BT 不再执行本地追击速度闭环。
- 姿态 topic `/ly/control/posture` 已并入主控制幀字段 `GimbalControlData.Posture`（单通道下发）。
- 下发全量规格见：`docs/sentry/lower_downlink_message_contract.md`。

## 4. 联调建议

- 联调前先确认 `/ly/control/vel` 是否只有预期发布者：  
  `ros2 topic info /ly/control/vel -v`
- 若缺少外部导航模块，不应将 `/ly/navi/*` 视为本仓缺件故障。  
