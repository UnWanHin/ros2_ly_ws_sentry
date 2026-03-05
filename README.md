# ros2_ly_ws_sentary 新接手阅读指南

这个仓库的文档已经统一收敛到 [`docs/`](docs/README.md)。

如果你是刚接手项目，按下面顺序读，能最快建立全局认知：

1. 项目总览与目录入口  
[`docs/README.md`](docs/README.md)
2. 系统链路与运行行为  
[`docs/architecture/message_and_link_flow.md`](docs/architecture/message_and_link_flow.md)  
[`docs/architecture/system_behavior.md`](docs/architecture/system_behavior.md)
3. 核心模块（建议按链路顺序）  
[`docs/modules/gimbal_driver.md`](docs/modules/gimbal_driver.md)  
[`docs/modules/detector.md`](docs/modules/detector.md)  
[`docs/modules/tracker_solver.md`](docs/modules/tracker_solver.md)  
[`docs/modules/predictor.md`](docs/modules/predictor.md)  
[`docs/modules/behavior_tree.md`](docs/modules/behavior_tree.md)
4. 哨兵决策与姿态（2026 重点）  
[`docs/sentry/sentry_decision_autoaim_manual.md`](docs/sentry/sentry_decision_autoaim_manual.md)  
[`docs/sentry/sentry_posture_system.md`](docs/sentry/sentry_posture_system.md)  
[`docs/sentry/sentry_posture_interface_change_2026-03-03.md`](docs/sentry/sentry_posture_interface_change_2026-03-03.md)
5. 上车前和实机检查  
[`docs/guides/config_setup_guide.md`](docs/guides/config_setup_guide.md)  
[`docs/guides/module_standalone_test.md`](docs/guides/module_standalone_test.md)  
[`docs/guides/preflight_checklist.md`](docs/guides/preflight_checklist.md)  
[`docs/guides/test_guide.md`](docs/guides/test_guide.md)

常用命令（本工作区）：

```bash
cd ~/ros2_ly_ws_sentary
colcon build
source install/setup.bash
./scripts/self_check_sentry.sh --skip-hz
```

说明：
- 根目录仅保留这份接手指南，其他文档已迁入 `docs/`。
- 比赛规则与通信协议 PDF 在 `docs/rules/`。
