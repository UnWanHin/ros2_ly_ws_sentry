# 0x0101 场地事件数据对齐记录（RM2026 V1.3.0）

## 1. 目标与范围

本次只对齐 `0x0101`（`ExtEventData`）位定义与命名，不改血量链路。

- 协议基准：`RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）`
- 改动范围：上位机 `ExtEventDataType` 结构定义、对齐说明文档
- 不改范围：`/ly/game/all.exteventdata` 与 `ly/gimbal/eventdata` 的消息类型（仍为 `uint32` 原值透传）

## 2. 上位机协议改动

### 2.1 代码改动

已同步修改两处 `ExtEventDataType`：

- `src/gimbal_driver/module/BasicTypes.hpp`
- `src/behavior_tree/module/BasicTypes.hpp`

改动内容：

1. 位宽由旧版 `3/3/6/7/9/2/2` 改为新版 `3/4/2/2/9/3/2/2/2/1/2`
2. 字段名改为与 V1.3.0 语义一致（见下表）

### 2.2 字段名映射（旧 -> 新）

- `SelfSupplyStatus` -> `SelfSupplyStatus`（保留）
- `SelfBuffStatus` -> `SelfSmallEnergyStatus` + `SelfLargeEnergyStatus`
- `SelfHighlandStatus` -> `SelfCentralHighlandStatus` + `SelfTrapezoidHighlandStatus`
- `SelfBaseShield` -> `EnemyLastDartHitTime`（语义更正）
- `LastDratTime` -> `EnemyLastDartHitTarget`（语义更正，且位宽变 3）
- `DartTarget` -> `CenterGainPointStatus`（语义更正）
- `GainPointstatus` -> `SelfFortressGainPointStatus` + `SelfOutpostGainPointStatus` + `SelfBaseGainPointStatus` + `Reserved`

### 2.3 当前上位机行为是否变化

当前行为本质不变：

- `gimbal_driver` 仍将 `ExtEventData` 作为 `uint32` 原值发布：
  - `/ly/game/all.exteventdata`
  - `ly/gimbal/eventdata`
- `behavior_tree` 仍按 `uint32` 接收并缓存 `extEventData`

结论：本次是**协议语义与命名对齐**，不是消息通道改造。

## 3. V1.3.0 位定义（0x0101）

| bit 范围 | 字段名（本仓库） | 说明 |
|---|---|---|
| 0-2 | `SelfSupplyStatus` | 己方补给区状态（含保留/RMUL 位） |
| 3-4 | `SelfSmallEnergyStatus` | 己方小能量机关激活状态 |
| 5-6 | `SelfLargeEnergyStatus` | 己方大能量机关激活状态 |
| 7-8 | `SelfCentralHighlandStatus` | 己方中央高地占领状态 |
| 9-10 | `SelfTrapezoidHighlandStatus` | 己方梯形高地占领状态 |
| 11-19 | `EnemyLastDartHitTime` | 对方飞镖最后一次命中己方前哨/基地时间 |
| 20-22 | `EnemyLastDartHitTarget` | 对方飞镖最后一次命中目标类型 |
| 23-24 | `CenterGainPointStatus` | 中心增益点占领状态（RMUL 适用） |
| 25-26 | `SelfFortressGainPointStatus` | 己方堡垒增益点占领状态 |
| 27-28 | `SelfOutpostGainPointStatus` | 己方前哨站增益点占领状态 |
| 29 | `SelfBaseGainPointStatus` | 己方基地增益点占领状态 |
| 30-31 | `Reserved` | 保留位 |

## 4. 下位机如何改并与上位机对齐

建议下位机使用 **bitmask 打包**（不要依赖 C/C++ bitfield 内存布局）：

```c
uint32_t ext_event_0101 = 0u;

ext_event_0101 |= ((uint32_t)(self_supply_status & 0x7u) << 0);   // bit0-2
ext_event_0101 |= ((uint32_t)(self_small_energy_status & 0x3u) << 3);  // bit3-4
ext_event_0101 |= ((uint32_t)(self_large_energy_status & 0x3u) << 5);  // bit5-6
ext_event_0101 |= ((uint32_t)(self_central_highland_status & 0x3u) << 7); // bit7-8
ext_event_0101 |= ((uint32_t)(self_trapezoid_highland_status & 0x3u) << 9); // bit9-10
ext_event_0101 |= ((uint32_t)(enemy_last_dart_hit_time & 0x1FFu) << 11); // bit11-19
ext_event_0101 |= ((uint32_t)(enemy_last_dart_hit_target & 0x7u) << 20); // bit20-22
ext_event_0101 |= ((uint32_t)(center_gain_point_status & 0x3u) << 23);   // bit23-24
ext_event_0101 |= ((uint32_t)(self_fortress_gain_point_status & 0x3u) << 25); // bit25-26
ext_event_0101 |= ((uint32_t)(self_outpost_gain_point_status & 0x3u) << 27);  // bit27-28
ext_event_0101 |= ((uint32_t)(self_base_gain_point_status & 0x1u) << 29); // bit29
// bit30-31 保留位写 0
```

对齐要求：

1. `bit1` 与 `bit30-31` 保留位默认写 `0`
2. 非 RMUL 场景下 `bit2`、`bit23-24` 按规则置默认值
3. 下位机和上位机统一使用小端字节序传输 `uint32`
4. 上位机当前按原值透传，不做字段拆分；后续若加决策拆分逻辑，需再同步协议文档

## 5. 联调检查清单

1. 串口侧抓包：确认 `ExtEventData` 原值随状态变化
2. ROS 侧检查：
   - `ros2 topic echo /ly/game/all`
   - `ros2 topic echo ly/gimbal/eventdata`
3. 对同一时刻数据，`/ly/game/all.exteventdata` 与 `ly/gimbal/eventdata.data` 必须一致
4. 用离线脚本按位拆解，核对每组状态值是否匹配裁判系统 UI

## 6. 备注

本次改动是**上位机协议定义层对齐**。  
血量与其他串口业务字段仍按现有链路工作，不在本次修改范围。
