# 哨兵姿态下位机对接说明（并入主控制幀）

> 下发全量协议（含角度/速度/火控/姿态）请优先看：`docs/sentry/lower_downlink_message_contract.md`

## 1. 目标

姿态控制（进攻/防御/移动）并入原有主控制幀，减少下位机协议分支复杂度。

上位机策略：
- 保留 `/ly/control/posture` ROS 接口不变
- 串口层将姿态写入 `GimbalControlData.Posture`
- 不再下发独立 `TypeID=7` 姿态幀
- 姿态回读沿用上行 `TypeID=6` 的 `ExtendData`，从 `Reserve_16` 高 8 bit 取值

---

## 2. 上位机当前行为（单通道）

输入 Topic：
- `/ly/control/posture` (`std_msgs/msg/UInt8`)
  - `0` 不请求切换（保留）
  - `1` 进攻
  - `2` 防御
  - `3` 移动

输出（主控制幀）：
- 结构体：`GimbalControlData`
- 姿态字段：`Posture`
- 切换重发：默认 3 次，间隔 20ms（可配置）
- 注意：当前下发不再封装额外 `TypeID`，下位机直接按 14B 主控制幀解析

主控制幀布局（14B）：
1. `HeadFlag` : `0x21` (`'!'`)
2. `Velocity.X`
3. `Velocity.Y`
4. `Yaw(float)`
5. `Pitch(float)`
6. `FireCode`
7. `Posture`
8. `Tail` : `0x00`

---

## 3. 下位机需要实现的内容

1. 主控制幀解析长度从 13B 升级为 14B（`Tail` 偏移后移）。  
2. 从主幀读取 `Posture` 字段并校验：只接受 `1/2/3`，`0` 视为保留值。  
3. 姿态映射到裁判链路 `0x0301 + data_cmd_id=0x0120` 的 `sentry_cmd bit21-22`：  
   - `1` 进攻  
   - `2` 防御  
   - `3` 移动  
4. 维护 `sentry_cmd_shadow`，只改 bit21-22，不重置其它位。  
5. 若当前固件已有独立 `TypeID=7` 分支，可仅作灰度兼容；当前上位机已不再发送该幀。  

---

## 3.1 下位机实现参考（可直接贴给固件同学）

### C 结构体（与上位机主幀对齐）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t head_flag;   // '!' = 0x21
    int8_t  vel_x;       // Velocity.X
    int8_t  vel_y;       // Velocity.Y
    float   yaw;         // GimbalAngles.Yaw
    float   pitch;       // GimbalAngles.Pitch
    uint8_t fire_code;   // FireCode
    uint8_t posture;     // 0/1/2/3
    uint8_t tail;        // 0x00
} gimbal_control_frame_t; // sizeof == 14
#pragma pack(pop)
```

### 字段偏移（0-based）

- `head_flag`: 0
- `vel_x`: 1
- `vel_y`: 2
- `yaw`: 3~6
- `pitch`: 7~10
- `fire_code`: 11
- `posture`: 12
- `tail`: 13

### 解析与执行建议

1. 先校验长度 `==14`，再校验 `head_flag==0x21 && tail==0x00`。  
2. `posture` 仅接受 `1/2/3`，`0` 直接忽略（保持当前姿态）。  
3. 对有效姿态做 5 秒切换冷却，避免频繁切换。  
4. 仅改 `sentry_cmd` 的 bit21-22，不覆盖其它位。  

### bit21-22 映射示例

```c
static inline uint32_t set_sentry_posture_bits(uint32_t sentry_cmd, uint8_t posture)
{
    // posture: 1=attack, 2=defense, 3=move
    const uint32_t mask = (0x3u << 21);               // bit21-22
    uint32_t val = ((uint32_t)(posture & 0x3u) << 21);
    return (sentry_cmd & ~mask) | val;
}
```

> 建议维护 `sentry_cmd_shadow`，每次只调用上述函数更新姿态位。

---

## 4. 状态回传约定

上位机继续从上行 `TypeID=6` 的 `ExtendData.Reserve_16` 高 8 位读取姿态并发布 `/ly/gimbal/posture`：
- `0` 未知
- `1` 进攻
- `2` 防御
- `3` 移动

建议下位机在姿态状态稳定后回填该字段，便于上位机闭环确认。姿态编码固定写入 `Reserve_16` 高 8 位。

---

## 5. 联调清单

1. 确认主包（angles/vel/firecode）链路稳定。  
2. 下位机升级到 14B 主幀解析并接入 `Posture` 字段。  
3. 手工发布 `/ly/control/posture`（1/2/3）。  
4. 验证：
   - 下位机日志收到姿态字段变化
   - 裁判状态姿态切换成功
   - `/ly/gimbal/posture` 回读一致
5. 若异常，先回滚到旧主幀解析版本，再定位字段偏移问题。  
