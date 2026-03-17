# 文档总索引（docs）

本目录是 `ros2_ly_ws_sentary` 的唯一文档入口，按“上手 -> 架构 -> 模块 -> 实机”组织。

## 目录结构

```text
docs/
├── README.md                    # 当前索引
├── architecture/                # 系统行为、消息链路
├── guides/                      # 配置、测试、上车前清单
├── modules/                     # 各 ROS 包说明
├── record/                      # 已落地的重要改动记录与调参记录
├── sentry/                      # 哨兵决策/姿态专项
├── reports/                     # 阶段性检查报告
├── references/                  # 参考材料（如 logger 文档）
├── rules/                       # 官方规则/通信协议 PDF
└── plans/                       # 历史方案与改造计划
```

## 推荐阅读顺序

1. 全局链路  
[architecture/message_and_link_flow.md](architecture/message_and_link_flow.md)
2. `/ly/control/angles` 专项链路追踪
[architecture/control_angles_data_flow.md](architecture/control_angles_data_flow.md)
3. 当前系统运行行为
[architecture/system_behavior.md](architecture/system_behavior.md)
4. 模块文档（建议按数据流）
[modules/gimbal_driver.md](modules/gimbal_driver.md)  
[modules/detector.md](modules/detector.md)  
[modules/tracker_solver.md](modules/tracker_solver.md)  
[modules/predictor.md](modules/predictor.md)  
[modules/behavior_tree.md](modules/behavior_tree.md)
5. 哨兵专项（2026 姿态机制）
[sentry/decision_runtime_behavior.md](sentry/decision_runtime_behavior.md)  
[sentry/sentry_decision_autoaim_manual.md](sentry/sentry_decision_autoaim_manual.md)  
[sentry/optional_items_demo_runbook.md](sentry/optional_items_demo_runbook.md)  
[sentry/showcase_demo_runbook.md](sentry/showcase_demo_runbook.md)  
[sentry/navi_debug_runbook.md](sentry/navi_debug_runbook.md)  
[sentry/sentry_posture_system.md](sentry/sentry_posture_system.md)  
[sentry/sentry_posture_interface_change_2026-03-03.md](sentry/sentry_posture_interface_change_2026-03-03.md)  
[sentry/posture_lower_firmware_integration.md](sentry/posture_lower_firmware_integration.md)  
[sentry/posture_firmware_integration_checklist.md](sentry/posture_firmware_integration_checklist.md)  
[sentry/lower_downlink_message_contract.md](sentry/lower_downlink_message_contract.md)
6. 落地执行与上车前检查
[guides/config_setup_guide.md](guides/config_setup_guide.md)  
[guides/auto_aim_tuning_handover.md](guides/auto_aim_tuning_handover.md)  
[guides/auto_aim_yamlization_checklist.md](guides/auto_aim_yamlization_checklist.md)  
[guides/external_topic_boundary.md](guides/external_topic_boundary.md)  
[guides/self_check_dual_suite.md](guides/self_check_dual_suite.md)  
[guides/module_standalone_test.md](guides/module_standalone_test.md)  
[guides/test_guide.md](guides/test_guide.md)  
[guides/preflight_checklist.md](guides/preflight_checklist.md)
7. 近期稳定性修复记录（接口不变）
[reports/stability_fix_no_interface_change_2026-03-05.md](reports/stability_fix_no_interface_change_2026-03-05.md)
[reports/self_check_status_2026-03-05.md](reports/self_check_status_2026-03-05.md)
[reports/repository_completeness_audit_2026-03-05.md](reports/repository_completeness_audit_2026-03-05.md)
[reports/full_link_audit_2026-03-05.md](reports/full_link_audit_2026-03-05.md)
8. 重要行为变更与调参记录
[record/autoaim_follow_fire_change_2026-03-18.md](record/autoaim_follow_fire_change_2026-03-18.md)  
[record/README.md](record/README.md)

## 分类说明

- `architecture/`
  - 系统级说明，先读它再看单包。
- `modules/`
  - 每个包的职责、topic、关键实现点。
- `sentry/`
  - 决策与姿态设计文档，和比赛规则最相关。
- `guides/`
  - 面向实机调试和赛前执行。
- `reports/`
  - 历史检查结果，可能与当前代码存在时间差。
- `record/`
  - 已落地的重要变更、现场调参与风险结论，优先写事实与当前建议。
- `plans/`
  - 方案设计记录，不代表最终已合入实现。

## 脚本说明入口

- `scripts/README.md`：脚本用途与命令总览（离车自检、上车自检、基础套件）。

## 历史与兼容文档

- 旧行为说明（保留对照）  
[architecture/system_behavior_v1.md](architecture/system_behavior_v1.md)
- 旧版射表标定说明（保留对照）  
[modules/shooting_table_calib_usage_legacy.md](modules/shooting_table_calib_usage_legacy.md)

## 文档维护约定

1. 新增文档统一放 `docs/`，禁止再落到仓库根目录。
2. 涉及接口变更（topic/msg/参数）时，至少同步更新：
   - 对应模块文档
   - `architecture/message_and_link_flow.md`
   - 本索引文件
3. 实机流程改动后，优先更新：
   - `guides/config_setup_guide.md`
   - `guides/preflight_checklist.md`
