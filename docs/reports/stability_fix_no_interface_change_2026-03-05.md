# 稳定性修复记录（接口不变）

日期：2026-03-05  
范围：`behavior_tree`、`outpost_hitter`、`predictor`、`buff_hitter`

## 目标

在不改变上下位机接口格式的前提下，修复已识别的系统级稳定性问题：

1. `outpost_hitter` 启动时序崩溃风险  
2. `behavior_tree` 开局等待卡死风险  
3. 高频日志导致的时序抖动风险  
4. `outpost` 目标 `status` 语义不一致

## 本次改动

1. `outpost_hitter` 启动时序修复  
文件：`src/outpost_hitter/outpost_hitter_node.cpp`  
- `PoseSolver` 改为在 `main` 设置 `SOLVER::global_solver_node` 后初始化。  
- 回调中增加 `solver` 就绪保护，避免空初始化阶段误调用。

2. `behavior_tree` 开局等待逻辑修复  
文件：`src/behavior_tree/include/Application.hpp`、`src/behavior_tree/src/SubscribeMessage.cpp`、`src/behavior_tree/src/WaitBeforeGame.cpp`  
- 新增“已收到云台角度消息”标志位。  
- `WaitBeforeGame` 改为等待“首包到达”而非“角度非零”。  
- 增加超时降级路径，避免无限阻塞。

3. `outpost` 火控状态语义一致化  
文件：`src/outpost_hitter/outpost_hitter_node.cpp`、`src/behavior_tree/src/SubscribeMessage.cpp`、`src/behavior_tree/src/GameLoop.cpp`  
- `outpost_hitter` 发布 `Target` 时显式写入 `status/buff_follow`。  
- `behavior_tree` 订阅 `ly_outpost_target` 时使用 `msg->status`。  
- 在 `PublishTogether()` 中按 `status` 决定是否执行翻转开火。

4. 热路径日志降载  
文件：`src/behavior_tree/src/GameLoop.cpp`、`src/behavior_tree/src/Application.cpp`、`src/predictor/src/motion_model.cpp`、`src/buff_hitter/src/BuffCalculator.cpp`、`src/outpost_hitter/outpost_hitter_node.cpp`  
- 高频 `Info/Warn/std::cout` 改为节流或 `Debug`。  
- `BT_DIAG` 输出改为编译开关 `BT_DIAG_CONSOLE`（默认关闭）。

## 接口影响评估

- ROS topic 名称：**无变化**  
- ROS msg 定义：**无变化**  
- 串口数据结构（`GimbalControlData` 等）：**无变化**  
- 包长/字段顺序：**无变化**

## 后续建议

1. 本次仅完成上位机稳定性修复；姿态下发到下位机串口仍待协议对接。  
2. 下位机接入姿态时，建议单独走“协议版本评审 + 联调窗口”，避免影响现有稳定链路。
