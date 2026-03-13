# 姿态并入主幀联调验收清单（下位机）

适用版本：单通道主控制幀（`GimbalControlData` 含 `Posture` 字段）

参考文档：
- `docs/sentry/lower_downlink_message_contract.md`
- `docs/sentry/posture_lower_firmware_integration.md`

---

## A. 协议改造前检查

- [ ] 下位机分支已备份（可快速回滚到旧 13B 解析）
- [ ] 串口主幀解析代码已定位到唯一入口
- [ ] 已确认当前上位机版本为单通道姿态方案（非 `TypeID=7` 独立幀）

---

## B. 协议解析改造（必须）

- [ ] 主控制幀长度改为 `14` 字节
- [ ] `head_flag` 校验为 `0x21`
- [ ] `tail` 校验偏移改为 byte `13`（值 `0x00`）
- [ ] `posture` 读取偏移为 byte `12`
- [ ] `posture` 仅接受 `1/2/3`，`0` 作为保留值忽略
- [ ] 旧 `TypeID=7` 分支已移除或仅保留灰度兼容

---

## C. 控制位映射改造（必须）

- [ ] 姿态正确映射到 `sentry_cmd` bit21-22
- [ ] 使用 `sentry_cmd_shadow` 更新，未覆盖其它控制位
- [ ] 姿态切换冷却（5 秒）生效

---

## D. 上行回传改造（闭环）

- [ ] 当前姿态已回填到上行 `TypeID=6` `ExtendData.Reserve_16` 高 8 位
- [ ] 姿态回填值范围为 `0/1/2/3`
- [ ] 上位机可稳定收到 `/ly/gimbal/posture`

---

## E. 联调命令与结果记录

建议上位机命令：

```bash
ros2 topic pub /ly/control/posture std_msgs/msg/UInt8 "{data: 1}" -1
ros2 topic pub /ly/control/posture std_msgs/msg/UInt8 "{data: 2}" -1
ros2 topic pub /ly/control/posture std_msgs/msg/UInt8 "{data: 3}" -1
```

记录：
- [ ] `data=1`（进攻）下位机执行成功
- [ ] `data=2`（防御）下位机执行成功
- [ ] `data=3`（移动）下位机执行成功
- [ ] 回读姿态与下位机实际状态一致
- [ ] 连续切换时冷却逻辑符合预期

---

## F. 异常场景回归（建议）

- [ ] 非法值（如 `4/255`）不会触发姿态切换
- [ ] 串口重连后姿态状态不乱跳
- [ ] 高负载下（持续角度/火控更新）姿态字段仍稳定生效
- [ ] 未出现误触发开火/旋转模式

---

## G. 最终签收

- [ ] 协议文档与固件实现一致
- [ ] 上下位机联调日志已归档
- [ ] 版本号/提交号已登记
- [ ] 可上车验证

签收信息：
- 固件负责人：`__________`
- 上位机负责人：`__________`
- 日期：`__________`
