# Reserve_32 上下位机对接规范（2026-04-06，更新于 2026-04-22）

> 文件名保留历史命名，但当前上位机实现是 `Reserve_32_1` 与 `Reserve_32_2` 共同承载 `/ly/gimbal/d_vel`。

## 1. 目的

对齐 `TypeID=6 (ExtendData)` 中 reserve 字段的当前真实对接方式，供上下位机联调。

## 2. 幀结构（不改长度）

```cpp
struct ExtendData {
    static constexpr auto TypeID = 6;
    std::uint16_t UWBAngleYaw;
    std::uint16_t Reserve_16;   // high8 = posture
    std::uint32_t Reserve_32_1; // used: high16
    std::uint32_t Reserve_32_2; // used: low16
};
```

## 3. 当前位分配（Little Endian）

- `Reserve_32_1` high16（byte2~3）: `d_vel_y_raw`（`int16`）
- `Reserve_32_2` low16（byte0~1）: `d_vel_x_raw`（`int16`）
- `Reserve_32_1` low16 与 `Reserve_32_2` high16：预留

## 4. 下位机打包方法

```c
int16_t d_vel_x_raw = ...;
int16_t d_vel_y_raw = ...;

uint32_t reserve_32_1 = 0;
uint32_t reserve_32_2 = 0;

reserve_32_1 |= ((uint32_t)((uint16_t)d_vel_y_raw) << 16); // high16
reserve_32_2 |= ((uint32_t)((uint16_t)d_vel_x_raw) << 0);  // low16
```

## 5. 上位机解析与输出

- Topic: `/ly/gimbal/d_vel`
- Msg: `gimbal_driver/msg/DVel`

```text
int16 x   // from Reserve_32_2 low16
int16 y   // from Reserve_32_1 high16
```

`behavior_tree` 订阅后直接保存 raw 值（当前不额外做 `*0.01` 缩放）。

## 6. 联调验证

```bash
ros2 topic echo /ly/gimbal/d_vel
```

检查点：

1. `x/y` 变化方向与下位机坐标定义一致
2. 大小端正确（无异常跳变）
3. `Reserve_16` high8 姿态位仍正常回读到 `/ly/gimbal/posture`

## 7. 预留扩展建议

优先复用：

1. `Reserve_32_1` low16
2. `Reserve_32_2` high16

扩展后同步更新：

- `src/gimbal_driver/main.cpp`
- 下游使用方（当前主要是 `behavior_tree`）
- 本文档与 `docs/sentry/current_upper_lower_data_mapping.md`
