# ExtendData `d_vel` Uplink Integration (2026-04-06, updated 2026-04-22)

> 文件名保留历史命名（`gimbal_yaw`），但当前实现已切换为 `/ly/gimbal/d_vel`。

## 1. Purpose

Keep `TypeID=6 (ExtendData)` frame length unchanged, and map reserve fields to one ROS2 topic:

- `/ly/gimbal/d_vel`

## 2. Uplink Frame Contract

```cpp
struct ExtendData {
    static constexpr auto TypeID = 6;
    std::uint16_t UWBAngleYaw; // 2 bytes
    std::uint16_t Reserve_16;  // high 8 bits used by posture feedback
    std::uint32_t Reserve_32_1;
    std::uint32_t Reserve_32_2;
};
```

## 3. Reserve Mapping (Little Endian)

- `Reserve_32_1` high16 -> `d_vel_y_raw` (`int16`)
- `Reserve_32_2` low16  -> `d_vel_x_raw` (`int16`)
- `Reserve_32_1` low16 and `Reserve_32_2` high16 are currently reserved

Current upper parsing in `src/gimbal_driver/main.cpp`:

```cpp
const auto reserve_32_1_high16_u = static_cast<std::uint16_t>((reserve_32_1_u32 >> 16) & 0xFFFFu);
const auto reserve_32_2_low16_u  = static_cast<std::uint16_t>(reserve_32_2_u32 & 0xFFFFu);
const auto d_vel_y_raw = static_cast<std::int16_t>(reserve_32_1_high16_u);
const auto d_vel_x_raw = static_cast<std::int16_t>(reserve_32_2_low16_u);
```

## 4. ROS Interface

Message: `gimbal_driver/msg/DVel.msg`

```text
int16 x
int16 y
```

Topic:

- `/ly/gimbal/d_vel` (`gimbal_driver/msg/DVel`)
- publish mapping: `x=d_vel_x_raw`, `y=d_vel_y_raw`

## 5. Upper Computer Behavior

### 5.1 gimbal_driver

- Parse `Reserve_32_1` high16 and `Reserve_32_2` low16
- Publish `/ly/gimbal/d_vel`
- Publish posture from `Reserve_16` high8 (`1/2/3` only)

### 5.2 behavior_tree

- Subscribe `/ly/gimbal/d_vel`
- Store:
  - `gimbalYawVelRaw` from `msg->x`
  - `gimbalYawAngleRaw` from `msg->y`
- Current code keeps float mirrors without extra scale:
  - `gimbalYawVelDegPerSec = static_cast<float>(gimbalYawVelRaw)`
  - `gimbalYawAngleDeg = static_cast<float>(gimbalYawAngleRaw)`

## 6. Lower Firmware Packing Guidance

```c
uint32_t reserve_32_1 = 0;
uint32_t reserve_32_2 = 0;

reserve_32_1 |= ((uint32_t)((uint16_t)d_vel_y_raw) << 16); // high16 used
reserve_32_2 |= ((uint32_t)((uint16_t)d_vel_x_raw) << 0);  // low16 used
```

`Reserve_16` high 8 bits posture feedback remains unchanged.

## 7. Runtime Check

```bash
ros2 topic echo /ly/gimbal/d_vel
```

Expected fields:

- `x`
- `y`
