# `ExtendData` 上行协议速查

## 1. 结论先看

当前下位机回传给上位机的 `ExtendData`：

- 幀类型：`TypeID=6`
- 方向：下位机 -> 上位机
- 在本仓库当前上位机解析/发布逻辑里，明确读到的字段只有两个：
  - `UWBAngleYaw`
  - `Reserve_16` 高 8 位（姿态回读）

按本仓库当前代码搜索，暂未发现其余字段的解析/发布引用：

- `Reserve_16` 低 8 位
- `Reserve_32_1`
- `Reserve_32_2`

代码依据：

- `src/gimbal_driver/module/BasicTypes.hpp`
- `src/gimbal_driver/main.cpp`

---

## 2. 结构体定义

当前代码中的定义如下：

```cpp
struct ExtendData {
    static constexpr auto TypeID = 6;
    std::uint16_t UWBAngleYaw;
    std::uint16_t Reserve_16;
    std::uint32_t Reserve_32_1;
    std::uint32_t Reserve_32_2;
};
```

来源：`src/gimbal_driver/module/BasicTypes.hpp`

在 `#pragma pack(push, 1)` 下，这个 payload 按当前定义是 12B。

---

## 3. 当前字段用途

| 字段 | 位范围 | 当前用途 | 上位机处理位置 |
|---|---|---|---|
| `UWBAngleYaw` | 16 bit | 自身 UWB 朝向角 | 发布到 `/ly/me/uwb_yaw` |
| `Reserve_16` | bit8~15（高8位） | 姿态状态回读 | 解析后发布到 `/ly/gimbal/posture` |
| `Reserve_16` | bit0~7（低8位） | 预留 | 无独立发布 |
| `Reserve_32_1` | low16（byte0~1） | 云台 yaw 角速度（`int16`, 0.01deg/s） | 发布到 `/ly/gimbal/gimbal_yaw` 的 `yaw_vel` 字段 |
| `Reserve_32_1` | high16（byte2~3） | 云台 yaw 当前角（`int16`, 0.01deg） | 发布到 `/ly/gimbal/gimbal_yaw` 的 `yaw_angle` 字段 |
| `Reserve_32_2` | 32 bit | 本仓库当前未发现解析/发布引用 | 无 |

当前上位机对 `Reserve_16` 的解释：

- `0`：未知 / 未实现 / 无效
- `1`：进攻
- `2`：防御
- `3`：移动

---

## 4. 当前上位机真实行为

`src/gimbal_driver/main.cpp` 里，`PubExtendData()` 当前做三件事：

1. 读取 `UWBAngleYaw`，发布 `/ly/me/uwb_yaw`
2. 读取 `Reserve_32_1`：
   - 低 16 位作为 yaw 角速度（`yaw_vel`）
   - 高 16 位作为 yaw 当前角（`yaw_angle`）
   并发布 `/ly/gimbal/gimbal_yaw`
3. 读取 `Reserve_16` 高 8 位，作为姿态值

对应逻辑要点：

- 只有 `1/2/3` 会被视为有效姿态并发布
- 如果下位机回传 `0`，当前实现不会发布新的 `/ly/gimbal/posture`
- `Reserve_32_1` 当前 32 位都已解析（低16=角速度，高16=当前角）
- `Reserve_32_2` 当前未解析/发布

这点要特别注意：

- 文档语义上 `0` 表示未知
- 但代码行为上，`0` 是“忽略，不发布”

也就是说，若下位机发 `0`，上位机现在看起来更像“没收到新的有效姿态”。

---

## 5. 给下位机同学的最简说明

如果只是对齐当前版本，下位机只需要保证：

1. 继续发送 `TypeID=6` 的 `ExtendData`
2. `UWBAngleYaw` 正常填写
3. 把姿态状态写入 `Reserve_16` 高 8 bit
4. `Reserve_32_1` 按约定填写：
   - low16: `yaw_vel_raw`（`int16`, `0.01deg/s`）
   - high16: `yaw_angle_raw`（`int16`, `0.01deg`）
5. `Reserve_32_2` 可先置 0

姿态编码方式（高 8 bit）：

```c
reserve_16 = ((uint16_t)(posture & 0xFFu) << 8) | reserve_8;
```

其中：

- `0` = 未知
- `1` = 进攻
- `2` = 防御
- `3` = 移动

---

## 6. 如果还想再塞别的数据

当前最安全的扩展顺序建议是：

1. 优先使用 `Reserve_32_2`
2. 再考虑 `Reserve_16` 低 8 位（高 8 位已承载 posture）

原因：

- `Reserve_16` 高 8 bit 已承载 posture（当前主约定）
- `Reserve_32_1` 当前 32 位都已有语义（角速度 + 当前角）
- `Reserve_32_2` 当前完全未被上位机解析，冲突最小
- 扩展时只需要同步修改：
  - `src/gimbal_driver/main.cpp`
  - 对应 topic / message 定义
  - 这份文档

---

## 7. 当前风险点

1. 若下位机错误写入高8位，姿态会被错误解析；建议把该字段纳入固件单元测试。
2. 若下位机希望把“未知姿态”明确回传给上位机，当前代码仍是“0 不发布”行为。
3. 若未来继续复用 `Reserve_16`，建议先固化位分配，避免与 posture 位冲突。

---

## 8. 相关文件

- `src/gimbal_driver/module/BasicTypes.hpp`
- `src/gimbal_driver/main.cpp`
- `docs/modules/gimbal_driver.md`
- `docs/sentry/posture_lower_firmware_integration.md`
- `docs/sentry/sentry_posture_interface_change_2026-03-03.md`
