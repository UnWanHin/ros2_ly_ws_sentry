# Gimbal Yaw Uplink Integration (2026-04-06)

## 1. Purpose

Keep `Reserve_32_1` as a 32-bit dual-field payload:

- low16 keeps **yaw angular velocity** (do not delete old yaw-vel lane)
- high16 replaces old pitch-vel lane with **current yaw angle**

Both are published through one ROS2 topic:

- `/ly/gimbal/gimbal_yaw`

## 2. Uplink Frame Contract

`ExtendData` frame length is unchanged:

```cpp
struct ExtendData {
    static constexpr auto TypeID = 6;
    std::uint16_t UWBAngleYaw; // 2 bytes
    std::uint16_t Reserve_16;  // high 8 bits used by posture feedback
    std::uint32_t Reserve_32_1;
    std::uint32_t Reserve_32_2;
};
```

## 3. Reserve_32_1 Mapping (Little Endian)

- byte0 + byte1: `yaw_vel_raw` (`int16`, unit `0.01 deg/s`)
- byte2 + byte3: `yaw_angle_raw` (`int16`, unit `0.01 deg`, recommended `[-180.00, 180.00)`)

Formulas:

- `yaw_vel_deg_s = yaw_vel_raw * 0.01`
- `yaw_angle_deg = yaw_angle_raw * 0.01`

Direction/sign is preserved by signed `int16`; positive direction definition is decided by lower firmware.

## 4. ROS Interface

`gimbal_driver/msg/GimbalYaw.msg`:

- `int16 yaw_vel` (0.01 deg/s)
- `int16 yaw_angle` (0.01 deg)

Topic:

- `/ly/gimbal/gimbal_yaw` (`gimbal_driver/msg/GimbalYaw`)

## 5. Upper Computer Changes

### 5.1 gimbal_driver

- Parse `Reserve_32_1` low16 -> `yaw_vel`
- Parse `Reserve_32_1` high16 -> `yaw_angle`
- Publish both fields on `/ly/gimbal/gimbal_yaw`

### 5.2 behavior_tree

- Subscribe `/ly/gimbal/gimbal_yaw`
- Store:
  - `gimbalYawVelRaw`, `gimbalYawVelDegPerSec`
  - `gimbalYawAngleRaw`, `gimbalYawAngleDeg`
- Write to blackboard:
  - `GimbalYawVelRaw`, `GimbalYawVelDegPerSec`
  - `GimbalYawAngleRaw`, `GimbalYawAngleDeg`

## 6. Lower Firmware Packing Guidance

1. `yaw_vel_raw = round(yaw_vel_deg_s * 100)` (`int16`)
2. `yaw_angle_raw = round(yaw_angle_deg * 100)` (`int16`)
3. Pack little-endian:
   - `Reserve_32_1[7:0]   = yaw_vel_raw low8`
   - `Reserve_32_1[15:8]  = yaw_vel_raw high8`
   - `Reserve_32_1[23:16] = yaw_angle_raw low8`
   - `Reserve_32_1[31:24] = yaw_angle_raw high8`
4. Keep `Reserve_16` high 8 bits posture feedback unchanged.

## 7. Runtime Check

```bash
ros2 topic echo /ly/gimbal/gimbal_yaw
```

Expected fields:

- `yaw_vel`
- `yaw_angle`
