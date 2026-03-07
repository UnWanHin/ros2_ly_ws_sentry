# 哨兵姿态接口改造说明（2026-03-03）

## 1. 目标

按现有链路风格新增一对姿态 Topic：

- 控制下发：`/ly/control/posture`
- 状态回读：`/ly/gimbal/posture`

并保持当前系统稳定，不直接破坏现有串口控制包长度。

---

## 2. 这次改动了什么

### 2.1 代码改动

- `src/behavior_tree/include/Topic.hpp`
  - 新增 Topic 定义：
    - `ly_control_posture` -> `/ly/control/posture` (`std_msgs/msg/UInt8`)
    - `ly_gimbal_posture` -> `/ly/gimbal/posture` (`std_msgs/msg/UInt8`)

- `src/behavior_tree/include/Application.hpp`
  - 新增姿态变量：
    - `postureCommand`（0/1/2/3）
    - `postureState`（0/1/2/3）
  - 新增发布者：
    - `pub_gimbal_posture_`（发布到 `/ly/control/posture`）
  - 新增函数声明：
    - `PubPostureControlData()`

- `src/behavior_tree/src/Application.cpp`
  - 初始化 `pub_gimbal_posture_`

- `src/behavior_tree/src/PublishMessage.cpp`
  - `PublishMessageAll()` 增加 `PubPostureControlData()`
  - 新增 `PubPostureControlData()`：
    - `postureCommand` 仅在 1/2/3 时下发
    - 0 视为“不下发姿态控制”

- `src/behavior_tree/src/SubscribeMessage.cpp`
  - 订阅 `/ly/gimbal/posture`，写入 `postureState`

- `src/gimbal_driver/main.cpp`
  - 新增 Topic 定义：
    - `/ly/control/posture`
    - `/ly/gimbal/posture`
  - 新增姿态缓存：
    - `postureCommand_`
    - `postureState_`
  - 新增 `IsValidPosture()` 与 `PublishPosture()`
  - 新增对 `/ly/control/posture` 的订阅与校验（0/1/2/3）
  - 在 `PubExtendData()` 中增加姿态回读发布逻辑：
    - 约定 `ExtendData.Reserve_16` 低 2 bit 作为姿态状态（仅 1/2/3 视为有效）

### 2.2 文档改动

- `docs/README.md`
  - Topic 速查表新增：
    - `/ly/control/posture`
    - `/ly/gimbal/posture`

- `docs/modules/gimbal_driver.md`
  - 增加姿态 Topic 在订阅/发布与注意事项中的说明

- `docs/modules/behavior_tree.md`
  - 增加姿态控制与回读 Topic 说明

---

## 3. 当前实现状态（非常重要）

### 已完成

- ROS 接口层已打通（Topic 命名、收发、状态变量、文档）
- 与现有 `angles/firecode/vel` 链路兼容
- `gimbal_driver` 已支持姿态独立下行幀（固定协议）：
  - 帧格式：`TypedMessage<1>`（`TypeID=7`，`Data[0]=posture`）
  - 固定重发：3 次，间隔 20ms

### 尚未完成（需下位机配合）

- 姿态控制不会写入 `GimbalControlData`（刻意保持旧包兼容）
- 需要下位机新增独立姿态帧解析（`TypeID=7`）
- 真实姿态状态回读依赖下位机把姿态值塞进 `ExtendData.Reserve_16` 低 2 bit

---

## 4. Topic 协议约定

### `/ly/control/posture`（上位机 -> gimbal_driver）

- 类型：`std_msgs/msg/UInt8`
- 值：
  - `0`: 不控制（保留值）
  - `1`: 进攻姿态
  - `2`: 防御姿态
  - `3`: 移动姿态

### `/ly/gimbal/posture`（gimbal_driver -> 上位机）

- 类型：`std_msgs/msg/UInt8`
- 值：
  - `0`: 未知/无效
  - `1`: 进攻姿态
  - `2`: 防御姿态
  - `3`: 移动姿态

---

## 5. 下位机电控需要做的事

详细版本见：
- `docs/sentry/posture_lower_firmware_integration.md`

## 5.1 指令下发链路

1. 接收上位机姿态指令（1/2/3）。
2. 做 5 秒切换冷却保护（避免频繁切换）。
3. 将姿态映射到裁判链路命令：
   - `0x0301` + `data_cmd_id=0x0120`
   - 仅修改 `sentry_cmd` 的 bit21-22（进攻/防御/移动）
4. 维持 `sentry_cmd_shadow`，不要每次重置整个 `uint32`，避免破坏其他控制位。

## 5.2 状态回读链路

1. 从裁判反馈读取当前姿态（建议用 `0x020D` 对应姿态位）。
2. 将当前姿态编码进上行 `ExtendData.Reserve_16` 低 2 bit（1/2/3，其他为0）。
3. 保持 `ExtendData` 总长度不变，确保上位机兼容。

## 5.3 失败处理建议

1. 姿态切换失败（冷却中/裁判拒绝/链路异常）要有错误码。
2. 若无法扩展当前上行包，可先把结果放到 `Reserve_32_1`（临时方案）并同步文档。

---

## 6. 联调建议

1. 启动 `gimbal_driver`，确认可见新 Topic：
   - `ros2 topic list | grep posture`
2. 手动发姿态：
   - `ros2 topic pub /ly/control/posture std_msgs/msg/UInt8 \"{data: 1}\" -1`
3. 观察回读：
   - `ros2 topic echo /ly/gimbal/posture`
4. 再与 `behavior_tree` 联调，使其在策略满足时写 `postureCommand`（1/2/3）。
