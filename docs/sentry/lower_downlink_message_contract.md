# 上位机下发协议总览（给下位机）

## 1. 目的与范围

本文档描述「上位机 -> 下位机」串口下发协议，供电控固件直接对接。  
不包含上行回传细节（上行姿态回读可参考 `ExtendData` 约定）。

当前下发采用**单通道单主幀**：角度/底盘速度/火控/姿态全部并入 `GimbalControlData`。
当前实现中：
- 上行：`TypedMessage` + `TypeID=0..6`
- 下行：直接写 `GimbalControlData` 原始主幀，不再发送独立 `TypeID=7`

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

## 3. 主控制幀（含姿态）

### 3.1 幀结构

对应结构体：`GimbalControlData`  
代码：`src/gimbal_driver/module/BasicTypes.hpp`

按 `#pragma pack(1)` 编排，长度 **14 bytes**：

1. `HeadFlag`（1B）=`'!'` (`0x21`)
2. `Velocity.X`（1B, int8）
3. `Velocity.Y`（1B, int8）
4. `GimbalAngles.Yaw`（4B, float32）
5. `GimbalAngles.Pitch`（4B, float32）
6. `FireCode`（1B, bitfield）
7. `Posture`（1B, uint8）
8. `Tail`（1B）=`0x00`

> 与旧版相比：新增 `Posture` 字段，`Tail` 偏移后移 1 字节。

### 3.2 ROS -> 字段映射

- `/ly/control/angles` (`gimbal_driver/msg/GimbalAngles`)
  - `yaw` -> `GimbalAngles.Yaw`
  - `pitch` -> `GimbalAngles.Pitch`
- `/ly/control/vel` (`gimbal_driver/msg/Vel`)
  - `x` -> `Velocity.X`
  - `y` -> `Velocity.Y`
- `/ly/control/firecode` (`std_msgs/msg/UInt8`)
  - `data` 原样写入 `FireCode` 字节
- `/ly/control/posture` (`std_msgs/msg/UInt8`)
  - `1` 进攻
  - `2` 防御
  - `3` 移动
  - `0` 保留（不请求姿态切换）

### 3.3 FireCode 位语义（1字节）

从低位到高位：
1. bit0-1: `FireStatus`（开火位，翻转触发，`0b00 <-> 0b11`）
2. bit2-3: `CapState`
3. bit4: `HoleMode`
4. bit5: `AimMode`
5. bit6-7: `Rotate`

下位机建议：
- 不要把 `FireStatus==1` 当作“持续开火”，按翻转沿触发。

### 3.4 姿态重发策略（上位机行为）

- 收到 `/ly/control/posture` 的 `1/2/3` 时，立即写入主幀 `Posture` 字段并下发
- 每次切换按参数重发（默认 `3` 次，间隔 `20ms`）
- 串口重连后会按当前姿态再次触发重发

### 3.5 姿态回读语义（联调约定）

- `/ly/gimbal/posture` 表示**下位机回读状态**，来源 `ExtendData.Reserve_16` 高 8 位（1/2/3）。
- 不建议将 `/ly/control/posture` 直接镜像回 `/ly/gimbal/posture`，否则会掩盖“已下发但未执行”的链路问题。
- 若下位机暂未实现回读，`/ly/gimbal/posture` 可保持未更新，上位机会按无回读路径处理（含重试/超时策略）。

---

## 4. 下位机实现要求

1. 主控制幀解析长度改为 **14B**，并更新 `Tail` 校验偏移。  
2. 解析 `Posture` 字段，仅接受 `1/2/3`，`0` 视为保留值。  
3. 将姿态映射到裁判链路 `0x0120 bit21-22`。  
4. 建议维护 `sentry_cmd_shadow`，只改 bit21-22，不重置其他控制位。  
5. 上行把姿态状态写入 `ExtendData.Reserve_16` 高 8 位。  

---

## 4.1 固件实现速查

### C 结构体

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t head_flag;
    int8_t  vel_x;
    int8_t  vel_y;
    float   yaw;
    float   pitch;
    uint8_t fire_code;
    uint8_t posture;
    uint8_t tail;
} gimbal_control_frame_t; // sizeof == 14
#pragma pack(pop)
```

### 关键偏移

- `posture` 在 byte `12`
- `tail` 在 byte `13`

### bit21-22 更新

```c
uint32_t mask = (0x3u << 21);
sentry_cmd_shadow = (sentry_cmd_shadow & ~mask) | (((uint32_t)posture & 0x3u) << 21);
```

---

## 5. 版本切换建议

- 若下位机仍在旧版 13B 主幀解析逻辑，必须先升级解析器再联调。  
- 若需要灰度切换，可短期保留双解析分支（13B/14B），确认稳定后再移除旧分支。  

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
