# 全链路审计（上位机↔下位机）- 2026-03-05

本报告只回答“链路是否打通/落地”，不做算法优劣评估。

## 1. 上位决策主链（打车）

`detector -> tracker_solver -> predictor -> behavior_tree -> gimbal_driver -> serial`

- `detector` 发布 `/ly/detector/armors`：已实现  
- `tracker_solver` 订阅并发布 `/ly/tracker/results`：已实现  
- `predictor` 订阅并发布 `/ly/predictor/target`：已实现  
- `behavior_tree` 订阅目标并发布 `/ly/control/angles`、`/ly/control/firecode`：已实现  
- `gimbal_driver` 订阅控制并写串口 `GimbalControlData`：已实现

结论：主打车链路在代码层闭环。

## 2. 专项链路（打符/前哨）

- 打符：`detector(/ly/ra/angle_image) -> buff_hitter(/ly/buff/target) -> behavior_tree -> gimbal_driver`  
  代码链路已闭环。
- 前哨：`detector(/ly/outpost/armors) -> outpost_hitter(/ly/outpost/target) -> behavior_tree -> gimbal_driver`  
  代码链路已闭环。

## 3. 下位机上行链路（裁判/状态回传）

`serial -> gimbal_driver LoopRead(TypeID 0..6) -> /ly/gimbal/* + /ly/game/* + /ly/team/buff + /ly/position/data + /ly/bullet/speed`

结论：上行解析与 ROS 发布完整，behavior_tree/predictor 有对应订阅。

## 4. 未完全落地/需特别注意项

1. （已闭环）姿态控制已并入串口主控制幀  
   当前 `GimbalControlData` 已包含 `Posture` 字段（14B 主幀），`/ly/control/posture` 会下发到下位机；  
   `/ly/gimbal/posture` 语义为下位机回读（当前代码读取 `ExtendData.Reserve_16` 高 8 位），不再建议作为命令镜像使用。

2. `/ly/control/vel` 由 `behavior_tree` 速度桥接发布  
   当前链路为：`/ly/navi/vel -> behavior_tree -> /ly/control/vel -> gimbal_driver`。

3. 导航 topic 缺少本仓库内执行消费者  
   behavior_tree 会发 `/ly/navi/*`，但仓库内未见对应导航执行节点（可能在外部仓库/下位机）。

4. 比赛开始门控  
   behavior_tree 会等待 `/ly/game/is_start`，若下位机未回传比赛状态，决策主循环不会进入正式阶段。

## 5. 本机运行校验说明

- `self_check_sentry.sh --static-only`：通过。  
- `--runtime-only --launch`：在当前沙箱环境触发 DDS/网络权限错误，不能据此判定实机链路故障。  
