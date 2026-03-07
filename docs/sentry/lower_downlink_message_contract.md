# 上位机下发协议总览（给下位机）

## 1. 目的与范围

本文档只描述「上位机 -> 下位机」串口下发协议，供电控固件直接对接。  
不包含上行回传协议细节（上行可参考 `ExtendData` 现有约定）。

当前下发包含两类消息：
1. 主控制幀（角度/底盘速度/火控）  
2. 姿态独立幀（进攻/防御/移动）

---

## 2. 通道说明

- 物理链路：同一串口全双工
- 下发网关节点：`gimbal_driver`
- 输入 ROS Topic：
  - `/ly/control/angles`
  - `/ly/control/vel`
  - `/ly/control/firecode`
  - `/ly/control/posture`

---

## 3. 主控制幀（固定旧协议）

### 3.1 幀结构

对应结构体：`GimbalControlData`  
代码：`src/gimbal_driver/module/BasicTypes.hpp`

按 `#pragma pack(1)` 编排，长度 13 bytes：

1. `HeadFlag`（1B）=`'!'` (`0x21`)
2. `Velocity.X`（1B, int8）
3. `Velocity.Y`（1B, int8）
4. `GimbalAngles.Yaw`（4B, float32）
5. `GimbalAngles.Pitch`（4B, float32）
6. `FireCode`（1B, bitfield）
7. `Tail`（1B）=`0x00`

### 3.2 ROS -> 字段映射

- `/ly/control/angles` (`gimbal_driver/msg/GimbalAngles`)
  - `yaw` -> `GimbalAngles.Yaw`
  - `pitch` -> `GimbalAngles.Pitch`
- `/ly/control/vel` (`gimbal_driver/msg/Vel`)
  - `x` -> `Velocity.X`
  - `y` -> `Velocity.Y`
- `/ly/control/firecode` (`std_msgs/msg/UInt8`)
  - `data` 原样写入 `FireCode` 字节

### 3.3 FireCode 位语义（1字节）

从低位到高位：
1. bit0-1: `FireStatus`（开火位，翻转触发，`0b00 <-> 0b11`）
2. bit2-3: `CapState`
3. bit4: `HoleMode`
4. bit5: `AimMode`
5. bit6-7: `Rotate`

下位机建议：
- 不要把 `FireStatus==1` 当作“持续开火”，按翻转沿触发。

---

## 4. 姿态独立幀（新增）

### 4.1 幀结构

对应结构体：`PostureControlMessage = TypedMessage<1>`  
代码：`src/gimbal_driver/module/BasicTypes.hpp`

长度 4 bytes：
1. `HeadFlag`（1B）=`'!'` (`0x21`)
2. `TypeID`（1B）=`7`
3. `Data[0]`（1B）=`posture`
4. `Tail`（1B）=`0x00`

`posture` 取值：
- `1` 进攻
- `2` 防御
- `3` 移动
- `0` 保留（不下发）

### 4.2 下发策略（当前实现）

- 收到 `/ly/control/posture` 的 `1/2/3` 时触发下发
- 每次切换固定重发 3 次
- 重发间隔 20ms
- 串口重连后会按当前姿态再触发一次下发

---

## 5. 下位机实现要求

1. 继续按原逻辑解析主控制幀（13B），不得改变旧幀兼容性。  
2. 新增 `TypeID=7` 分支，仅解析 1-byte posture payload。  
3. posture 非 `1/2/3` 直接丢弃。  
4. 将姿态映射到裁判链路 `0x0120 bit21-22`（建议维护 `sentry_cmd_shadow`，只改该位段）。  
5. 保持上行姿态回读到 `ExtendData.Reserve_16` 低 2 bit（上位机已接入）。  

---

## 6. 联调最小命令

1. 发角度：
```bash
ros2 topic pub /ly/control/angles gimbal_driver/msg/GimbalAngles "{yaw: 10.0, pitch: 2.0}" -1
```

2. 发速度：
```bash
ros2 topic pub /ly/control/vel gimbal_driver/msg/Vel "{x: 10, y: -10}" -1
```

3. 发火控字节（示例 `0x03`）：
```bash
ros2 topic pub /ly/control/firecode std_msgs/msg/UInt8 "{data: 3}" -1
```

4. 发姿态（防御）：
```bash
ros2 topic pub /ly/control/posture std_msgs/msg/UInt8 "{data: 2}" -1
```
