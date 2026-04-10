# Reserve_32_1 上下位機對接規範（2026-04-06）

## 1. 目的

本文只描述 `TypeID=6 (ExtendData)` 中 `Reserve_32_1` 的對接方式，供下位機與上位機聯調使用。

當前約定是：

- `Reserve_32_1` 低 16 位：雲台 yaw 角速度
- `Reserve_32_1` 高 16 位：雲台 yaw 當前角度

不再承載 pitch 速度。

## 2. 幀結構（不改長度）

```cpp
struct ExtendData {
    static constexpr auto TypeID = 6;
    std::uint16_t UWBAngleYaw; // 保持原語義
    std::uint16_t Reserve_16;  // 高8位仍給 posture
    std::uint32_t Reserve_32_1; // 本文重點
    std::uint32_t Reserve_32_2; // 目前保留
};
```

## 3. Reserve_32_1 位分配

按小端序（little-endian）解析：

- byte0~1（低16位）：`yaw_vel_raw`（`int16`，單位 `0.01 deg/s`）
- byte2~3（高16位）：`yaw_angle_raw`（`int16`，單位 `0.01 deg`）

換算關係：

- `yaw_vel_deg_s = yaw_vel_raw * 0.01`
- `yaw_angle_deg = yaw_angle_raw * 0.01`

角度推薦區間：

- `yaw_angle_deg` 建議由下位機限制在 `[-180.00, 180.00)`

注意：

- 符號位有效（`int16`），方向正負定義由下位機坐標系決定，但要固定並在下位機文檔說明。

## 4. 下位機打包方法

### 4.1 量化

```c
int16_t yaw_vel_raw   = (int16_t)lroundf(yaw_vel_deg_s * 100.0f);
int16_t yaw_angle_raw = (int16_t)lroundf(yaw_angle_deg * 100.0f);
```

### 4.2 組包（小端）

```c
uint32_t reserve_32_1 = 0;
reserve_32_1 |= ((uint32_t)((uint16_t)yaw_vel_raw)   << 0);   // low16
reserve_32_1 |= ((uint32_t)((uint16_t)yaw_angle_raw) << 16);  // high16
```

等價字節視角：

- `reserve_32_1[7:0]`    = `yaw_vel_raw` low8
- `reserve_32_1[15:8]`   = `yaw_vel_raw` high8
- `reserve_32_1[23:16]`  = `yaw_angle_raw` low8
- `reserve_32_1[31:24]`  = `yaw_angle_raw` high8

## 5. 上位機當前解析與輸出

### 5.1 ROS topic

- Topic: `/ly/gimbal/gimbal_yaw`
- Msg: `gimbal_driver/msg/GimbalYaw`

```text
int16 yaw_vel
int16 yaw_angle
float32 yaw_vel_deg_s
float32 yaw_angle_deg
```

其中：

- `yaw_vel / yaw_angle` 是原始量化值（`0.01` 精度）
- `yaw_vel_deg_s / yaw_angle_deg` 是上位機已換算後的人類可讀值

### 5.2 行為樹側

上位機行為樹收到後會做 `*0.01` 換算，並寫入黑板：

- `GimbalYawVelRaw`
- `GimbalYawVelDegPerSec`
- `GimbalYawAngleRaw`
- `GimbalYawAngleDeg`

## 6. 聯調驗證

### 6.1 查看原始回讀

```bash
ros2 topic echo /ly/gimbal/gimbal_yaw
```

示例：

```text
yaw_vel: 1234
yaw_angle: -9000
yaw_vel_deg_s: 12.34
yaw_angle_deg: -90.0
```

對應：

- 角速度 `12.34 deg/s`
- 角度 `-90.00 deg`

### 6.2 常見錯誤排查

1. 角度/角速度單位錯：發成 rad 或 deg 而不是 `0.01deg`/`0.01deg/s`。
2. 大小端反了：會出現數值跳變或方向錯誤。
3. 發了浮點二進制：本協議要求 `int16` 量化值，不是 float 位模式。
4. 角度不做範圍管理：建議下位機固定在 `[-180,180)`，避免跨界抖動。

## 7. 其他字段影響

- `Reserve_16` 高8位 posture 規則不變。
- `Reserve_32_2` 目前未使用，可留作後續擴展。
