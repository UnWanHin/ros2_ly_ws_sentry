# 哨兵姿态下位机对接说明（独立下行通道）

> 下发全量协议（含角度/速度/火控/姿态）请优先看：`docs/sentry/lower_downlink_message_contract.md`

## 1. 目标

在不破坏现有云台/底盘/火控串口主包的前提下，新增姿态控制（进攻/防御/移动）下发能力。

上位机策略：
- 保留 `GimbalControlData` 旧包不变
- 姿态走独立下行幀（固定协议）

---

## 2. 上位机当前行为（已实现）

输入 Topic：
- `/ly/control/posture` (`std_msgs/msg/UInt8`)
  - `0` 不控制
  - `1` 进攻
  - `2` 防御
  - `3` 移动

输出（独立下行幀）：
- 帧类型：`PostureControlMessage`（`TypedMessage<1>`）
- `TypeID=7`（固定）
- payload：`Data[0] = posture`
- 固定重发：3 次，间隔 20ms

二进制布局（默认）：
1. `HeadFlag` : `0x21` (`'!'`)
2. `TypeID`   : `0x07`
3. `Data[0]`  : posture（1/2/3）
4. `Tail`     : `0x00`

说明：
- 该通道不会改旧主包格式（`GimbalControlData` 不变）。
- 下位机只需新增 `TypeID=7` 分支，不会影响旧包解析。

---

## 3. 下位机需要实现的内容

1. 在串口接收分发中新增 `TypeID=7` 处理分支。  
2. 仅接受 posture=1/2/3，其他值丢弃并记录日志。  
3. 加 5 秒切换冷却（防抖与防误触）。  
4. 将姿态映射到裁判链路 `0x0301 + data_cmd_id=0x0120` 的 `sentry_cmd bit21-22`：  
   - `1` 进攻  
   - `2` 防御  
   - `3` 移动  
5. 维护 `sentry_cmd_shadow`，只改 bit21-22，不重置其它位。  

---

## 4. 状态回传约定

上位机已支持从 `ExtendData.Reserve_16` 低 2 bit 读取姿态并发布 `/ly/gimbal/posture`：
- `0` 未知
- `1` 进攻
- `2` 防御
- `3` 移动

建议下位机在姿态状态稳定后回填该字段，便于上位机闭环确认。

---

## 5. 联调清单

1. 确认旧主包（angles/vel/firecode）链路稳定。  
2. 下位机上线 `TypeID=7` 解析与 0x0120 映射。  
3. 手工发布 `/ly/control/posture`（1/2/3）。  
4. 验证：
   - 下位机日志收到姿态命令
   - 裁判状态姿态切换成功
   - `/ly/gimbal/posture` 回读一致
5. 若异常，回退策略为暂时不发 `/ly/control/posture`（旧主包链路仍正常）。  
