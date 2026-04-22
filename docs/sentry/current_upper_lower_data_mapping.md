# 上下位机当前对接数据总表

## 1. 说明

这份文档只描述**当前上位机代码实际实现的串口对接逻辑**，范围以 `gimbal_driver` 为准：

- 上位机 -> 下位机：`src/gimbal_driver/main.cpp` 里的订阅与下发
- 下位机 -> 上位机：`src/gimbal_driver/main.cpp` 里的串口解析与 topic 发布
- 底层结构定义：`src/gimbal_driver/module/BasicTypes.hpp`

不讨论下位机固件内部怎么采集这些值，只说明：

1. 当前串口上到底有哪些结构
2. 每个字段对应什么数据
3. 上位机收到/发出后怎么解析
4. 最后映射到哪些 ROS topic

---

## 2. 总体链路

当前 `gimbal_driver` 的串口模板实例是：

```cpp
IODevice<TypedMessage<sizeof(GimbalData)>, GimbalControlData>
```

含义：

- 上行：下位机 -> 上位机，使用 `TypedMessage`
- 下行：上位机 -> 下位机，直接写 `GimbalControlData`

也就是说：

1. 上行是**带 `TypeID` 的分型幀**
2. 下行是**不带独立 `TypeID` 的主控制幀**

---

## 3. 下行：上位机 -> 下位机

## 3.1 当前下发结构

结构体：`GimbalControlData`

```cpp
struct GimbalControlData
{
    std::uint8_t HeadFlag{ '!' };
    VelocityType Velocity;
    GimbalAnglesType GimbalAngles;
    FireCodeType FireCode;
    std::uint8_t Posture{0};
    std::uint8_t Tail{ 0 };
};
```

按当前定义，主控制幀长度是 **14B**。

### 3.1.1 字节布局

| byte offset | 字段 | 类型 | 来源 | 当前上位机写法 |
|---|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定值 | `'!'` / `0x21` |
| 1 | `Velocity.X` | `int8` | `/ly/control/vel.x` | 直接 `static_cast<int8_t>` |
| 2 | `Velocity.Y` | `int8` | `/ly/control/vel.y` | 直接 `static_cast<int8_t>` |
| 3~6 | `GimbalAngles.Yaw` | `float` | `/ly/control/angles.yaw` | 直接写 `float` |
| 7~10 | `GimbalAngles.Pitch` | `float` | `/ly/control/angles.pitch` | 直接写 `float` |
| 11 | `FireCode` | `uint8` | `/ly/control/firecode.data` | 原样写 1 字节 |
| 12 | `Posture` | `uint8` | `/ly/control/posture.data` | 仅接受 `0/1/2/3` |
| 13 | `Tail` | `uint8` | 固定值 | `0x00` |

### 3.1.2 ROS 输入与字段映射

| ROS topic | ROS 消息字段 | 串口字段 | 备注 |
|---|---|---|---|
| `/ly/control/angles` | `yaw` | `GimbalAngles.Yaw` | 云台目标 yaw |
| `/ly/control/angles` | `pitch` | `GimbalAngles.Pitch` | 云台目标 pitch |
| `/ly/control/vel` | `x` | `Velocity.X` | 底盘速度 x |
| `/ly/control/vel` | `y` | `Velocity.Y` | 底盘速度 y |
| `/ly/control/firecode` | `data` | `FireCode` | 整字节原样写入 |
| `/ly/control/posture` | `data` | `Posture` | `0=保留, 1=进攻, 2=防御, 3=移动` |

### 3.1.3 `FireCode` 位定义

`FireCode` 在代码里是位域结构，但下发时上位机按**整字节**写入：

| bit | 名称 | 含义 |
|---|---|---|
| 0~1 | `FireStatus` | 开火位，代码注释约定为翻转触发 |
| 2~3 | `CapState` | 电容状态 |
| 4 | `HoleMode` | 钻洞模式 |
| 5 | `AimMode` | 瞄准模式 |
| 6~7 | `Rotate` | 小陀螺档位 |

### 3.1.4 `Posture` 当前规则

当前代码行为：

1. `0` 允许写入，表示“不请求姿态切换 / 保留值”
2. `1/2/3` 为有效姿态
3. 非 `0/1/2/3` 会被上位机直接丢弃并报警告
4. 有效姿态会按参数做重发

当前重发参数默认值：

- `io_config/posture_repeat_count = 3`
- `io_config/posture_repeat_interval_ms = 20`

---

## 4. 上行：下位机 -> 上位机

## 4.1 当前上行总封装

上行读取类型是：

```cpp
TypedMessage<sizeof(GimbalData)>
```

按当前代码，`GimbalData` payload 是 12B，因此当前上行总封装可理解为：

| 字段 | 长度 |
|---|---:|
| `HeadFlag` | 1B |
| `TypeID` | 1B |
| `Data` | 12B |
| `Tail` | 1B |

总长度：**15B**

### 4.1.1 `TypeID` 分发表

| TypeID | 结构体 | 解析函数 |
|---|---|---|
| `0` | `GimbalData` | `PubGimbalData()` |
| `1` | `GameData` | `PubGameData()` |
| `2` | `HealthMyselfData` | `PubHealthMyselfData()` |
| `3` | `HealthEnemyData` | `PubHealthEnemyData()` |
| `4` | `RFIDAndBuffData` | `PubRFIDAndBuffData()` |
| `5` | `PositionData` | `PubPositionData()` |
| `6` | `ExtendData` | `PubExtendData()` |

---

## 5. 各 `TypeID` 的字段、解析与 topic

## 5.1 `TypeID=0` - `GimbalData`

结构：

```cpp
struct GimbalData
{
    GimbalAnglesType GimbalAngles;
    VelocityType Velocity;
    FireCodeType FireCode;
    std::uint8_t CapV;
};
```

### 5.1.1 字段映射

| 串口字段 | 解析方式 | 发布 topic | ROS 字段 |
|---|---|---|---|
| `GimbalAngles.Yaw` | 直接读 `float` | `/ly/gimbal/angles` | `yaw` |
| `GimbalAngles.Pitch` | 直接读 `float` | `/ly/gimbal/angles` | `pitch` |
| `Velocity.X` | 直接读 `int8` | `/ly/gimbal/vel` | `x` |
| `Velocity.Y` | 直接读 `int8` | `/ly/gimbal/vel` | `y` |
| `FireCode` | 整字节重解释 | `/ly/gimbal/firecode` | `data` |
| `CapV` | 直接读 `uint8` | `/ly/gimbal/capV` | `data` |

---

## 5.2 `TypeID=1` - `GameData`

结构：

```cpp
struct GameData
{
    GameCodeType GameCode;
    std::uint16_t AmmoLeft;
    std::uint16_t TimeLeft;
    std::uint16_t SelfHealth;
    std::uint32_t ExtEventData;
};
```

### 5.2.1 `GameCode` 位定义

| bit | 字段 | 当前含义 |
|---|---|---|
| 0 | `IsGameBegin` | 比赛开始标志 |
| 1 | `HeroPrecaution` | 英雄预警 |
| 2 | `IsMyTeamRed` | 我方是否红方 |
| 3~8 | `EnemyOutpostHealth` | 敌方前哨站血量分度值 |
| 9~14 | `SelfOutpostHealth` | 我方前哨站血量分度值 |
| 15 | `IsReturnedHome` | 是否回家 |

当前代码对前哨站血量的处理是：

- `EnemyOutpostHealth * 25` -> `/ly/enemy/op_hp`
- `SelfOutpostHealth * 25` -> `/ly/me/op_hp`

### 5.2.2 `ExtEventData` 位定义

`BasicTypes.hpp` 中定义了位域，但当前 `gimbal_driver` 没有拆 bit 发布，只做原样透传：

| bit | 字段 |
|---|---|
| 0~2 | `SelfSupplyStatus` |
| 3~4 | `SelfSmallEnergyStatus` |
| 5~6 | `SelfLargeEnergyStatus` |
| 7~8 | `SelfCentralHighlandStatus` |
| 9~10 | `SelfTrapezoidHighlandStatus` |
| 11~19 | `EnemyLastDartHitTime` |
| 20~22 | `EnemyLastDartHitTarget` |
| 23~24 | `CenterGainPointStatus` |
| 25~26 | `SelfFortressGainPointStatus` |
| 27~28 | `SelfOutpostGainPointStatus` |
| 29 | `SelfBaseGainPointStatus` |
| 30~31 | `Reserved` |

### 5.2.3 字段映射

| 串口字段 | 解析方式 | 发布 topic | ROS 字段 / 备注 |
|---|---|---|---|
| `GameCode` | 整体重解释成 `uint16` | `/ly/game/all` | `gamecode` |
| `AmmoLeft` | 直接读 `uint16` | `/ly/game/all` | `ammoleft` |
| `AmmoLeft` | 直接读 `uint16` | `/ly/me/ammo_left` | `data` |
| `TimeLeft` | 直接读 `uint16` | `/ly/game/all` | `timeleft` |
| `TimeLeft` | 直接读 `uint16` | `/ly/game/time_left` | `data` |
| `SelfHealth` | 直接读 `uint16` | `/ly/game/all` | `selfhealth` |
| `ExtEventData` | 整体转 `uint32` | `/ly/game/all` | `exteventdata` |
| `ExtEventData` | 整体转 `uint32` | `ly/gimbal/eventdata` | `data`，注意当前 topic 字符串无前导 `/` |
| `GameCode.EnemyOutpostHealth` | `* 25` | `/ly/enemy/op_hp` | `data` |
| `GameCode.HeroPrecaution` | 直接读 bit | `/ly/me/is_precaution` | `data` |
| `GameCode.IsGameBegin` | 直接读 bit | `/ly/game/is_start` | `data` |
| `GameCode.IsMyTeamRed` | 直接读 bit | `/ly/me/is_team_red` | `data` |
| `GameCode.IsReturnedHome` | 直接读 bit | `/ly/me/is_at_home` | `data` |
| `GameCode.SelfOutpostHealth` | `* 25` | `/ly/me/op_hp` | `data` |

---

## 5.3 `TypeID=2` - `HealthMyselfData`

结构：

```cpp
struct HealthMyselfData {
    std::uint16_t HeroMyself;
    std::uint16_t EngineerMyself;
    std::uint16_t Infantry1Myself;
    std::uint16_t Infantry2Myself;
    std::uint16_t BaseMyself;
    std::uint16_t SentryMyself;
};
```

### 5.3.1 字段映射

| 串口字段 | 发布 topic | ROS 字段 |
|---|---|---|
| `HeroMyself` | `/ly/me/hp` | `hero` |
| `EngineerMyself` | `/ly/me/hp` | `engineer` |
| `Infantry1Myself` | `/ly/me/hp` | `infantry1` |
| `Infantry2Myself` | `/ly/me/hp` | `infantry2` |
| `BaseMyself` | `/ly/me/hp` | `reserve` |
| `SentryMyself` | `/ly/me/hp` | `sentry` |
| `BaseMyself` | `/ly/me/base_hp` | `data` |

注意：当前 `Health.msg` 的 `reserve` 字段，实际上被上位机填的是 `BaseMyself`。

---

## 5.4 `TypeID=3` - `HealthEnemyData`

结构：

```cpp
struct HealthEnemyData {
    std::uint16_t HeroEnemy;
    std::uint16_t EngineerEnemy;
    std::uint16_t Infantry1Enemy;
    std::uint16_t Infantry2Enemy;
    std::uint16_t BaseEnemy;
    std::uint16_t SentryEnemy;
};
```

### 5.4.1 字段映射

| 串口字段 | 发布 topic | ROS 字段 |
|---|---|---|
| `HeroEnemy` | `/ly/enemy/hp` | `hero` |
| `EngineerEnemy` | `/ly/enemy/hp` | `engineer` |
| `Infantry1Enemy` | `/ly/enemy/hp` | `infantry1` |
| `Infantry2Enemy` | `/ly/enemy/hp` | `infantry2` |
| `BaseEnemy` | `/ly/enemy/hp` | `reserve` |
| `SentryEnemy` | `/ly/enemy/hp` | `sentry` |
| `BaseEnemy` | `/ly/enemy/base_hp` | `data` |

注意：当前 `Health.msg` 的 `reserve` 字段，实际上被上位机填的是 `BaseEnemy`。

---

## 5.5 `TypeID=4` - `RFIDAndBuffData`

结构：

```cpp
struct BuffType{
    std::uint8_t reserve;
    std::uint8_t RecoveryBuff;
    std::uint8_t CoolingBuff;
    std::uint8_t DefenceBuff;
    std::uint8_t VulnerabilityBuff;
    std::uint16_t AttackBuff;
    std::uint8_t RemainingEnergy;
};

struct RFIDAndBuffData{
    BuffType BuffStatus;
    std::uint32_t RFIDStatus;
};
```

### 5.5.1 字段映射

| 串口字段 | 发布 topic | ROS 字段 / 备注 |
|---|---|---|
| `RFIDStatus` | `/ly/me/rfid` | `data` |
| `BuffStatus.RecoveryBuff` | `/ly/team/buff` | `recoverybuff` |
| `BuffStatus.CoolingBuff` | `/ly/team/buff` | `coolingbuff` |
| `BuffStatus.DefenceBuff` | `/ly/team/buff` | `defencebuff` |
| `BuffStatus.VulnerabilityBuff` | `/ly/team/buff` | `vulnerabilitybuff` |
| `BuffStatus.AttackBuff` | `/ly/team/buff` | `attackbuff` |
| `BuffStatus.RemainingEnergy` | `/ly/team/buff` | `remainingenergy` |

注意：

- `BuffStatus.reserve` 当前没有被发布

---

## 5.6 `TypeID=5` - `PositionData`

结构：

```cpp
struct PositionType{
    uint8_t CarId;
    int16_t X;
    int16_t Y;
};

struct PositionData{
    PositionType Friend;
    PositionType Enemy;
    uint16_t BulletSpeed;
};
```

### 5.6.1 字段映射

| 串口字段 | 解析方式 | 发布 topic | ROS 字段 / 备注 |
|---|---|---|---|
| `Friend.CarId` | 直接读 | `/ly/position/data` | `friendcarid` |
| `Friend.X` | 直接读 | `/ly/position/data` | `friendx` |
| `Friend.Y` | 直接读 | `/ly/position/data` | `friendy` |
| `Enemy.CarId` | 直接读 | `/ly/position/data` | `enemycarid` |
| `Enemy.X` | 直接读 | `/ly/position/data` | `enemyx` |
| `Enemy.Y` | 直接读 | `/ly/position/data` | `enemyy` |
| `BulletSpeed` | `/100.0f` | `/ly/bullet/speed` | `data` |

### 5.6.2 特殊逻辑

如果：

```cpp
data.Friend.CarId == 7
```

则上位机还会额外发布：

- topic：`/ly/me/uwb_pos`
- 类型：`UInt16MultiArray`
- 内容：`[Friend.X, Friend.Y]`

注意当前代码行为：

1. `Friend.X/Y` 原始类型是 `int16_t`
2. 发布 `/ly/me/uwb_pos` 时，代码做了 `static_cast<std::uint16_t>`
3. 所以这里是“按当前实现直接转换后发布”，不是重新定义过坐标系

---

## 5.7 `TypeID=6` - `ExtendData`

结构：

```cpp
struct ExtendData {
    std::uint16_t UWBAngleYaw;
    std::uint16_t Reserve_16;
    std::uint32_t Reserve_32_1;
    std::uint32_t Reserve_32_2;
};
```

### 5.7.1 字段映射

| 串口字段 | 解析方式 | 发布 topic | 备注 |
|---|---|---|---|
| `UWBAngleYaw` | 直接读 `uint16` | `/ly/me/uwb_yaw` | 自身朝向角 |
| `Reserve_16` bit8~15（高8位） | 按 `uint8` 姿态值解析 | `/ly/gimbal/posture` | 仅 `1/2/3` 才发布 |
| `Reserve_16` bit0~7（低8位） | 当前未解析 | 无 | 预留 |
| `Reserve_32_1` low16（byte0~1） | 当前未解析 | 无 | 预留 |
| `Reserve_32_1` high16（byte2~3） | 按 `int16` 原始值解析 | `/ly/gimbal/d_vel` | 写入 `msg.y` |
| `Reserve_32_2` low16（byte0~1） | 按 `int16` 原始值解析 | `/ly/gimbal/d_vel` | 写入 `msg.x` |
| `Reserve_32_2` high16（byte2~3） | 当前未解析 | 无 | 预留 |

### 5.7.2 姿态回读规则

当前代码约定：

| 值 | 含义 |
|---|---|
| `0` | 未知 / 无效 |
| `1` | 进攻 |
| `2` | 防御 |
| `3` | 移动 |

但要注意当前代码行为：

- 文档语义上 `0` 表示未知
- 代码里只有 `1/2/3` 会发布 `/ly/gimbal/posture`
- 如果下位机发 `0`，当前上位机不会主动发布一个新的 `0`

---

## 6. 当前已对接数据汇总

## 6.1 上位机 -> 下位机已对接

| 数据项 | 串口字段 | 来源 topic |
|---|---|---|
| 云台目标角 yaw | `GimbalControlData.GimbalAngles.Yaw` | `/ly/control/angles` |
| 云台目标角 pitch | `GimbalControlData.GimbalAngles.Pitch` | `/ly/control/angles` |
| 底盘速度 x | `GimbalControlData.Velocity.X` | `/ly/control/vel` |
| 底盘速度 y | `GimbalControlData.Velocity.Y` | `/ly/control/vel` |
| 火控字节 | `GimbalControlData.FireCode` | `/ly/control/firecode` |
| 姿态指令 | `GimbalControlData.Posture` | `/ly/control/posture` |

## 6.2 下位机 -> 上位机已对接

| TypeID | 数据项 | 串口字段 | 输出 topic |
|---|---|---|---|
| `0` | 云台角 | `GimbalData.GimbalAngles` | `/ly/gimbal/angles` |
| `0` | 底盘速度 | `GimbalData.Velocity` | `/ly/gimbal/vel` |
| `0` | 火控状态字节 | `GimbalData.FireCode` | `/ly/gimbal/firecode` |
| `0` | 电容值 | `GimbalData.CapV` | `/ly/gimbal/capV` |
| `1` | 比赛摘要 | `GameData` | `/ly/game/all` |
| `1` | 子弹余量 | `GameData.AmmoLeft` | `/ly/me/ammo_left` |
| `1` | 比赛剩余时间 | `GameData.TimeLeft` | `/ly/game/time_left` |
| `1` | 敌方前哨站血量 | `GameCode.EnemyOutpostHealth` | `/ly/enemy/op_hp` |
| `1` | 我方前哨站血量 | `GameCode.SelfOutpostHealth` | `/ly/me/op_hp` |
| `1` | 英雄预警 | `GameCode.HeroPrecaution` | `/ly/me/is_precaution` |
| `1` | 比赛开始标志 | `GameCode.IsGameBegin` | `/ly/game/is_start` |
| `1` | 我方颜色 | `GameCode.IsMyTeamRed` | `/ly/me/is_team_red` |
| `1` | 回家标志 | `GameCode.IsReturnedHome` | `/ly/me/is_at_home` |
| `1` | 场地事件原始值 | `GameData.ExtEventData` | `/ly/game/all`, `ly/gimbal/eventdata` |
| `2` | 我方各兵种血量 | `HealthMyselfData` | `/ly/me/hp` |
| `2` | 我方基地血量 | `HealthMyselfData.BaseMyself` | `/ly/me/base_hp` |
| `3` | 敌方各兵种血量 | `HealthEnemyData` | `/ly/enemy/hp` |
| `3` | 敌方基地血量 | `HealthEnemyData.BaseEnemy` | `/ly/enemy/base_hp` |
| `4` | RFID | `RFIDAndBuffData.RFIDStatus` | `/ly/me/rfid` |
| `4` | 增益状态 | `RFIDAndBuffData.BuffStatus.*` | `/ly/team/buff` |
| `5` | 位置数据 | `PositionData.Friend/Enemy` | `/ly/position/data` |
| `5` | 自身 UWB 坐标 | `PositionData.Friend.X/Y` | `/ly/me/uwb_pos` |
| `5` | 弹速 | `PositionData.BulletSpeed` | `/ly/bullet/speed` |
| `6` | 自身朝向 | `ExtendData.UWBAngleYaw` | `/ly/me/uwb_yaw` |
| `6` | 扩展回读 `d_vel.x` | `ExtendData.Reserve_32_2` low16（`int16` 原始值） | `/ly/gimbal/d_vel` |
| `6` | 扩展回读 `d_vel.y` | `ExtendData.Reserve_32_1` high16（`int16` 原始值） | `/ly/gimbal/d_vel` |
| `6` | 姿态回读 | `ExtendData.Reserve_16` 高 8 位 | `/ly/gimbal/posture` |

---

## 7. 当前未真正拆出的字段

下面这些字段在结构定义里有，但在本仓库当前 `gimbal_driver` 中没有单独解析/发布：

| 结构 | 字段 | 当前状态 |
|---|---|---|
| `RFIDAndBuffData.BuffStatus` | `reserve` | 未发布 |
| `ExtendData` | `Reserve_16` 剩余位（除 posture 编码） | 未解析 |
| `ExtendData` | `Reserve_32_1` low16、`Reserve_32_2` high16 | 未解析 |
| `ExtEventDataType` | 各 bit 子字段 | 仅整体透传，未单独拆 topic |

---

## 8. 代码定位

- 串口结构：`src/gimbal_driver/module/BasicTypes.hpp`
- 下发订阅入口：`src/gimbal_driver/main.cpp` 的 `GenSubs()`
- 上行解析分发入口：`src/gimbal_driver/main.cpp` 的 `LoopRead()`
- `TypeID=0`：`PubGimbalData()`
- `TypeID=1`：`PubGameData()`
- `TypeID=2`：`PubHealthMyselfData()`
- `TypeID=3`：`PubHealthEnemyData()`
- `TypeID=4`：`PubRFIDAndBuffData()`
- `TypeID=5`：`PubPositionData()`
- `TypeID=6`：`PubExtendData()`
