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

## 离车复测结果（2026-03-18）

本次使用离车方案 A 复测：

- `ros2 launch detector auto_aim.launch.py offline:=true use_gimbal:=false output:=log`

观测 topic：

- `/ly/detector/armors`
- `/ly/tracker/results`
- `/ly/predictor/target`

### 1. 清日志前

离车实测结果：

- `/ly/detector/armors` 约 `6.8 Hz`
- `/ly/tracker/results` 约 `9.6 Hz`
- `/ly/predictor/target` 约 `4.4 Hz`

结论：

- 主问题不是 `behavior_tree`
- 更不是电控 PID
- 因为本次测试根本没有启动 `behavior_tree`，也没有接实车
- 慢是 `detector -> tracker_solver -> predictor` 这一段自己就慢

### 2. 注释高频日志后

本次注释的高频日志位置：

- `src/tracker_solver/src/tracker.cpp`
- `src/tracker_solver/include/car_tracker/tracker_matcher_with_whole_car.hpp`
- `src/predictor/src/controller.cpp`
- `src/predictor/src/motion_model.cpp`

重新离车实测结果：

- `/ly/detector/armors` `24.100 Hz`
- `/ly/tracker/results` `24.172 Hz`
- `/ly/predictor/target` `24.122 Hz`

结论：

- 之前“纯慢”的直接主因就是高频日志刷屏
- 注释这些日志后，前半段链路频率已恢复到约 `24 Hz`
- 因此如果上车后仍有“纯慢”现象，应继续区分：
  - `/ly/control/angles` 是否也慢
  - `/ly/gimbal/angles` 是否比 `/ly/control/angles` 更慢

### 3. 当前 `status=false` 的判断

离车抓到的 `/ly/predictor/target` 样本中：

- `yaw/pitch` 正常输出
- `status=false`

结合当前控制逻辑：

- 若 `calcPitchYawWithShootTable()` 失败，则通常会直接返回当前云台角，离车默认更接近 `0/0`
- 实测样本里的 `yaw/pitch` 已经是有效解算角，不像是早退分支
- 当前 `controller` 在成功解算后，只会因为两类条件把 `valid` 重新打回 `false`
  - `unstable_track`
  - `yaw_jump_reject`

当前离车样本中的 `yaw` 幅值不大，工程上更像：

- `status=false` 主要由 `unstable_track` 触发

也就是说：

- “慢”已经通过注释高频日志显著改善
- “火控没恢复”则更像是 `predictor/controller` 仍然认为当前轨迹不够稳定

### 4. 第二轮修正与复测

在第一轮“去高频日志”之后，继续检查到两类问题：

#### A. `detector` 参数 missing 日志是误导性的

现象：

- 离车 launch 时 `detector` 会连续打印：
  - `Parameter 'detector_config.*' missing, fallback default applied.`

但实际现象又说明：

- 视频路径、模型路径、离线模式都仍然能工作

根因：

- `auto_aim_common/include/RosTools/RosTools.hpp` 中的 `ROSNode` 没有开启：
  - `allow_undeclared_parameters(true)`
  - `automatically_declare_parameters_from_overrides(true)`

结果：

- launch 传进来的参数 override 存在
- 但 `has_parameter()` 在显式声明前返回 `false`
- 导致 `LoadParamCompat()` 误报 missing

修正后结果：

- 这批参数 missing 日志已消失
- 后续离车调试时不会再被误导到“YAML 根本没吃进去”

#### B. `tracker` 时序与 whole-car 动态模型存在单位问题

检查发现：

- `tracker_matcher.hpp` 里注释写的是 `20 ms` 基准
- 但代码用的是 `(time - last_time).seconds() / timeRatio`
- `tracker_matcher_with_whole_car.hpp` 里日志写的是 `ms`
- 但代码同样使用 `seconds()`
- 与此同时 `maxOmega = 10 * 2 * PI / 1000.0`
- 明显是按“毫秒时间基”设计的限幅

修正方式：

- `tracker_matcher.hpp` 改成以毫秒计算 `dt`
- `tracker_matcher_with_whole_car.hpp` 也改成以毫秒计算 `dt`
- `maxOmega` 保持原本的毫秒量纲设计

#### C. `tracker` 主路径恢复为原时序 matcher，失败时安全回退

现象：

- 当前 `Tracker::getTrackResult()` 其实绕开了原来的 `getArmorTrackResult(time, gimbalAngle)`
- 直接走 `initArmorTrackResult(gimbalAngle)`

影响：

- 更像“当前帧初始化 + whole-car 再补装甲板 ID”
- 而不是完整利用时序 matcher 做装甲板跟踪

修正方式：

- 先恢复 `getArmorTrackResult(time, gimbalAngle)` 作为主路径
- 如果 `TrackerMatcher` 输出尺寸异常，则安全回退到当前帧初始化结果

这样做的目的：

- 优先恢复原有时序 matcher 的连续性
- 同时避免因历史 matcher corner case 直接丢结果

### 5. 第二轮离车复测结果

复测命令与第一次相同：

- `ros2 launch detector auto_aim.launch.py offline:=true use_gimbal:=false output:=log`

结果：

- `/ly/detector/armors` 提升到约 `34 Hz`
- `/ly/tracker/results` 提升到约 `34 Hz`
- `/ly/predictor/target` 提升到约 `34 Hz`

这说明：

- 第一轮主要是日志瓶颈
- 第二轮则进一步修掉了 tracker 的时间尺度问题和参数层误导问题

### 6. 当前残余风险

虽然第二轮修正后频率继续提升，但仍捕获到一次：

- `TrackerMatcher fallback for car_id ... result size != center size`

说明：

- 原 `TrackerMatcher` 仍然存在偶发重复/缺失 ID 的历史 corner case
- 当前代码已改为“警告 + 安全回退”，不会直接丢掉整帧结果
- 但这仍是后续值得继续清理的 tracking 语义问题

### 7. 第三轮修正与复测

继续排查后，确认还存在两类会影响跟随表现、但不属于 fire gate 的问题：

#### A. `tracker` 时间单位写法与注释不一致

检查发现：

- `tracker_matcher.hpp` 的 `timeRatio` 注释明确写的是 `20 ms`
- whole-car matcher 日志也写的是 `delta_time: ... ms`
- 但代码实际一直在用 `seconds()`

修正方式：

- `tracker_matcher.hpp` 改成：
  - `dt = (time - last_time).seconds() * 1000.0 / timeRatio`
- `tracker_matcher_with_whole_car.hpp` 改成：
  - `dt = (time - last_time).seconds() * 1000.0`

同时保留原本 `maxOmega = 10 * 2 * PI / 1000.0` 的毫秒量纲设计。

#### B. `tracker` 主路径恢复后存在历史 corner case

恢复 `getArmorTrackResult(time, gimbalAngle)` 为主路径后，发现原 matcher 偶发会出现：

- `result size != center size`

说明：

- 旧 matcher 不是完全无问题
- 不能直接无保护地恢复

当前处理：

- 原 matcher 作为主路径
- 一旦当前 car_id 的 matcher 结果尺寸异常，立即警告并回退到当前帧初始化结果

这样做的取舍：

- 尽量保留时序 matcher 的连续性
- 又不会因为历史 corner case 直接丢装甲板结果

### 8. 第三轮离车复测结果

复测命令保持不变：

- `ros2 launch detector auto_aim.launch.py offline:=true use_gimbal:=false output:=log`

结果：

- `/ly/detector/armors` 提升到约 `34 Hz`
- `/ly/tracker/results` 提升到约 `34 Hz`
- `/ly/predictor/target` 提升到约 `34 Hz`

同时确认：

- `detector` 启动时不再出现一串误导性的参数 missing 日志
- 但运行期仍捕获过一次：
  - `TrackerMatcher fallback for car_id ... result size != center size`

结论：

- 当前“纯慢”问题已经不再是主矛盾
- `detector -> tracker_solver -> predictor` 的离车链路频率已恢复到正常区间
- 剩余更值得关注的是：
  - 旧 matcher 的 corner case
  - `status=false` 背后的稳定性门控

### 9. 第四轮架构增强（连续数据流 + predictor timer）

本轮不是再去拉长 BT hold，而是保留 `behavior_tree` 收口结构，只增强前半段数据流连续性：

#### A. `detector`：空观测也继续发布

调整点：

- 即使当前帧 `Detect` 失败，也继续发布一帧 `Armors`
- 即使当前帧 `Filter` 失败，也继续发布一帧 `Armors`
- 这帧中：
  - `header / yaw / pitch` 仍然有效
  - `armors` 可以为空
  - `cars` 尽量保留当前可得的车框

目的：

- 让下游收到的是“空观测”
- 而不是“节点没消息”

#### B. `tracker_solver`：空观测也继续发布

调整点：

- 取消 `msg->armors.empty()` 时直接 `return`
- 即使这帧没有装甲板，也照样发布 `Trackers`
- 仅在 `track_results.first` 非空时才调用 `solver->solve_all()`

目的：

- 让 `predictor` 能持续接收到时序 heartbeat
- 而不是上游一空就直接断流

#### C. `predictor`：从“回调即发布”改为“回调 update + timer publish”

调整点：

- `tracker` 回调只做：
  - 更新内部模型
  - 记录最近一次云台角与 header
  - 记录最近一次有效观测时间
- 新增固定 `10ms` 的 wall timer
  - 定频发布 `/ly/predictor/target`
  - 定频发布时再调用 `controller->control()`

#### D. `predictor`：新增短时 coast 语义

当前策略：

- 若仍有预测结果：继续给角
- 若观测短时过期但仍处于 coast timeout 内：允许继续输出角
- 若预测也不存在且已超时：
  - 发布 `status=false`
  - 发布 `yaw=NaN`
  - 发布 `pitch=NaN`

这样做的目的：

- `BT` 在失锁时不再误把“当前云台角”当成有效目标角
- 又不会靠拉长 `kLostTargetHold` 去硬撑

### 10. 第四轮离车验证结果

离车验证中，`/ly/predictor/target` 已经表现为固定频率输出层：

- `/ly/predictor/target` 约 `90.9 Hz`

同时抓到一条失锁样本：

- `status=false`
- `yaw=NaN`
- `pitch=NaN`

这说明：

- predictor timer 已经生效
- 失锁状态会明确下发为无效角，而不是继续伪造“当前云台角就是目标角”

说明：

- 这轮离车验证已经足够证明“数据流和输出频率”明显变强
- 但是否“更锁”仍需要上车或至少完整 BT 链下继续验证体感

## 结论

这次调整保留了原有链路设计：

- `detector -> tracker_solver -> predictor -> behavior_tree -> gimbal_driver`

但修正了一个关键设计问题：

- `不能打` 不应该等于 `不能跟`

当前版本适合作为“保守、可上线验证”的最小改动基线。
