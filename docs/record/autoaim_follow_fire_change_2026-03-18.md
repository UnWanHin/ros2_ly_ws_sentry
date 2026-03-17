# 自瞄秒跟/禁乱打改动记录（2026-03-18）

## 背景

本次调整针对一个具体问题：

- 画面中已经持续框住目标
- 但云台没有直接锁到 `predictor` 解算角
- 表现为“慢慢跟”“一格一格跟”

根因不是单纯的 PID 或下位机速度问题，而是上位机链路里把“是否允许跟随”和“是否允许开火”绑定到了同一个 `Target.status` 上。

旧行为等价于：

- `不能开火` = `不能接收 predictor 角度`

这会导致：

- 目标虽然还在框里
- 但只要 `predictor/controller` 暂时把 `status` 判成 `false`
- `behavior_tree` 就不再跟随该角度
- 最终表现成“偶尔跳一下，再停住”

## 本次改动目标

目标保持不变：

- 不乱打
- 秒跟

实现策略：

- 跟随和开火解耦
- 保留现有 topic / msg / 节点结构
- 不做大规模接口重构

## 核心设计结论

当前约定改为：

- `BT 跟随`：只要收到有限的 `yaw/pitch`，就允许接收并下发该角度
- `BT 开火`：仍然由 `FireStatus` 决定

也就是说：

- `有角度` 不再等于 `允许开火`
- `不能开火` 也不再阻止 `秒跟`

## 实际代码改动

### 1. behavior_tree：跟随与开火解耦

修改文件：

- `src/behavior_tree/src/SubscribeMessage.cpp`
- `src/behavior_tree/src/GameLoop.cpp`

关键变化：

- `ly_predictor_target` 与 `ly_outpost_target` 不再要求 `msg.status == true` 才接收角度
- 只要 `yaw/pitch` 为有限值，就更新 `Angles`
- `msg.status` 继续写入 `FireStatus`

当前行为：

- `Valid` 表示这一帧角度可用于跟随
- `FireStatus` 表示是否允许开火

### 2. behavior_tree：缩短丢目标后的锁角保持时间

修改文件：

- `src/behavior_tree/src/GameLoop.cpp`

关键变化：

- `kLostTargetHold` 从 `2000ms` 改为 `120ms`

目的：

- 保留 1 到 2 帧抖动缓冲
- 避免长时间粘住旧目标角

### 3. predictor：保留不稳定目标的 prediction 供跟随

修改文件：

- `src/predictor/src/predictor.cpp`

关键变化：

- 不再因为 `car->Stable()` 为假就整车不出 prediction
- 不稳定目标仍然参与预测
- 但 `prediction.stable` 仍然保留，供 fire gate 使用

目的：

- 允许目标在“尚未满足开火条件”时仍然先跟上去

### 4. 热路径日志降级

修改文件：

- `src/detector/detector_node.cpp`
- `src/predictor/src/predictor.cpp`
- `src/predictor/src/controller.cpp`
- `src/tracker_solver/src/solver.cpp`

关键变化：

- 删除 `detector` 每帧 `WARN`
- 高频 `INFO/WARN` 改为 `DEBUG`

目的：

- 降低 `screen` 输出带来的频率损耗
- 避免“本来能秒跟，但输出把它拖成阶梯跟”

## 当前关键参数与常量

以下参数分成两类：

- 运行期配置参数：优先从 YAML / JSON 调
- 编译期代码常量：只有明确需要时再改代码

### A. 运行期配置参数

#### 1. 开火门控

来源：

- `src/behavior_tree/Scripts/ConfigJson/*.json`

参数：

- `AimDebug.FireRequireTargetStatus`

含义：

- `true`：开火仍然要求 `FireStatus == true`
- `false`：BT 允许忽略该门控直接开火

建议：

- 比赛/实车默认保持 `true`
- 不建议为了“秒跟”把它改成 `false`

#### 2. predictor/controller 射表与预测延迟

来源：

- `scripts/config/auto_aim_config_competition.yaml`
- `src/detector/config/auto_aim_config.yaml`

参数：

- `controller_config.shoot_delay`
- `controller_config.shoot_table_adjust.enable`
- `controller_config.shoot_table_adjust.pitch.*`
- `controller_config.shoot_table_adjust.yaw.*`

含义：

- 控制前视时间与射表补偿

影响：

- 这些参数主要影响“跟上后打得准不准”
- 不应拿它们解决“有框但不跟”的问题

#### 3. solver / 相机外参与内参

来源：

- `solver_config.camera_offset`
- `solver_config.camera_rotation`
- `solver_config.camera_intrinsic_matrix`
- `solver_config.camera_distortion_coefficients`

影响：

- 会影响空间解算结果是否跳变
- 若角度明显飘或高低左右存在系统误差，优先检查这里

#### 4. detector 模式参数

来源：

- `detector_config.*`

重点项：

- `detector_config.use_video`
- `detector_config.use_ros_bag`
- `detector_config.debug_team_blue`
- `detector_config.draw`
- `detector_config.show`

影响：

- 主要决定输入源、可视化和调试形态
- 不直接决定“秒跟/乱打”

### B. 当前仍在代码中的关键常量

#### 1. 丢目标保持时间

文件：

- `src/behavior_tree/src/GameLoop.cpp`

常量：

- `kLostTargetHold = 120ms`

调法：

- 若仍明显粘住旧目标，可尝试降到 `80ms`
- 若画面偶发掉帧导致抖动，可尝试升到 `150ms`

建议范围：

- `80ms` 到 `150ms`

#### 2. yaw 跳变拒绝阈值

文件：

- `src/predictor/src/controller.cpp`

逻辑：

- `yaw_diff > 80°` 时，当前结果判为 `valid = false`

影响：

- 太小：容易跟不上大角度切换
- 太大：容易在错误目标/错误解算时乱跳

建议：

- 先保持不动
- 只有在确认角度本身可信但频繁被拒时再调

#### 3. 子弹速度硬编码

文件：

- `src/predictor/src/controller.cpp`
- `src/gimbal_driver/main.cpp`

当前行为：

- 实际上仍被写死为 `23.0`

风险：

- 会影响弹道计算与开火判断
- 不是“秒跟”问题的第一优先级
- 但会影响“跟上后打得稳不稳”

## 微调建议

### 只想让它更快跟上

优先顺序：

1. 先确认热路径日志已经降下来
2. 观察 `kLostTargetHold` 是否仍然偏长
3. 再判断 `yaw_jump_reject` 是否过严

不建议：

- 直接乱改射表参数去解决“慢跟”

### 只想让它更不容易乱打

优先顺序：

1. 保持 `FireRequireTargetStatus = true`
2. 保持 `prediction.stable` 参与 fire gate
3. 不要把 `status` 的 fire 语义删掉

不建议：

- 为了“看起来更灵敏”直接放开 fire 门控

### 想继续提升到“秒跟且稳打”

下一阶段最值得做的事：

1. 把 `aim_valid` 和 `fire_ready` 正式拆成两个字段
2. 统一 `tracker_solver` 两条解算路径的 pitch 旋转符号
3. 去掉 `bullet_speed = 23.0` 的硬编码

## 验证建议

改动后现场应重点观察：

1. `/ly/predictor/target`
   - 即使 `status = 0`
   - `yaw/pitch` 是否仍连续输出
2. `/ly/control/angles`
   - 是否开始直接跟随 predictor 的角度
   - 是否不再长时间停在当前云台角
3. `/ly/control/firecode`
   - 是否仍只在满足开火条件时翻转

## 结论

这次调整保留了原有链路设计：

- `detector -> tracker_solver -> predictor -> behavior_tree -> gimbal_driver`

但修正了一个关键设计问题：

- `不能打` 不应该等于 `不能跟`

当前版本适合作为“保守、可上线验证”的最小改动基线。
