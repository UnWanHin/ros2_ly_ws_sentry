# 0x0209 RFID Status（32-bit）对齐记录（RM2026 V1.3.0）

日期：2026-04-23  
范围：`TypeID=4 (RFIDAndBuffData)`

## 1. 本次目标

- 对齐 `RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）` 的 `0x0209`。
- 在现有自定义上行 12B 固定 payload 约束下，仅接入 `rfid_status` 的低 32 位（bit0-31）。
- `rfid_status_2`（额外 8 位）本次不并入 `TypeID=4`，后续放到其他通道。

## 2. 上位机改动（已完成）

### 2.1 结构体语义对齐（不改大小）

以下结构保持不变：

```cpp
struct RFIDAndBuffData {
    BuffType BuffStatus;   // 8B
    uint32_t RFIDStatus;   // 4B -> 对齐 0x0209 rfid_status(bit0-31)
};
```

结论：`sizeof(RFIDAndBuffData)==12`，仍满足当前固定 12B 分型帧。

### 2.2 代码侧保护

在以下模块新增尺寸断言，防止后续误改为非 12B：

- `src/gimbal_driver/module/BasicTypes.hpp`
- `src/behavior_tree/module/BasicTypes.hpp`
- `src/buff_hitter/module/BasicTypes.hpp`

断言：

```cpp
static_assert(sizeof(RFIDAndBuffData) == sizeof(GimbalData), "TypeID=4 payload must stay 12B");
```

### 2.3 文档同步

已更新：

- `docs/sentry/current_upper_lower_data_mapping.md`
- `docs/modules/gimbal_driver.md`

新增 `RFIDStatus(bit0-31)` 含义表，并注明 `rfid_status_2` 未接入当前 `TypeID=4`。

## 3. 下位机打包对齐要求

## 3.1 发送到上位机 TypeID=4 的数据约定

- `BuffStatus`：继续按现有 `0x0204` 对应字段填充（8B）。
- `RFIDStatus`：填充 `0x0209` 的 `rfid_status`（低 32 位）。
- 不要在 `TypeID=4` 内追加第 13 字节（`rfid_status_2`），否则会破坏固定帧解析。

## 3.2 关于 `rfid_status_2`

- `rfid_status_2` 属于 `0x0209` 的扩展 8 位。
- 建议通过以下任一方式后续接入：
  - 新增一个 TypeID 专门承载扩展字节。
  - 复用其他已规划的扩展通道（由上下位机统一约定）。

## 4. 位定义（仅本次接入范围）

`RFIDStatus` 对应 `0x0209 rfid_status` 的 bit0-31。
完整 bit 语义见：

- `docs/sentry/current_upper_lower_data_mapping.md`（5.5.2 节）

## 5. 兼容性结论

- ROS topic 契约不变：`/ly/me/rfid` 仍是 `UInt32`。
- `behavior_tree` 当前仍按原样透传 `rfidStatus`，不做位拆解，行为不变。
- 本次属于协议语义对齐，不引入链路长度变化。
