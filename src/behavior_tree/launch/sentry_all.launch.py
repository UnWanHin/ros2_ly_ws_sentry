#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
哨兵整链路启动入口（比赛/联调主入口）。

职责：
- 拉起 gimbal_driver / detector / tracker_solver / predictor / outpost_hitter / buff_hitter / behavior_tree。
- 支持通过 offline 参数统一覆盖“虚拟串口 + 视频回放”。

注意：
- behavior_tree 会接管 /ly/control/*，调试外部控制脚本时不要并行启动。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction, SetLaunchConfiguration, Shutdown
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    def normalize_mode(raw: str) -> str:
        normalized = (raw or "").strip().lower()
        if normalized in ("1", "league"):
            return "league"
        if normalized in ("3", "showcase", "demo"):
            return "showcase"
        return "regional"

    def resolve_mode_defaults(context):
        mode_raw = LaunchConfiguration("mode").perform(context)
        competition_profile_raw = LaunchConfiguration("competition_profile").perform(context).strip()
        bt_config_file_raw = LaunchConfiguration("bt_config_file").perform(context).strip()

        if mode_raw.strip():
            mode_kind = normalize_mode(mode_raw)
        elif bt_config_file_raw.endswith("showcase_competition.json"):
            mode_kind = "showcase"
        elif competition_profile_raw.strip().lower() == "league":
            mode_kind = "league"
        else:
            mode_kind = "regional"

        resolved_profile = competition_profile_raw or ("league" if mode_kind == "league" else "regional")
        if bt_config_file_raw:
            resolved_bt_config = bt_config_file_raw
        elif mode_kind == "league":
            resolved_bt_config = "Scripts/ConfigJson/league_competition.json"
        elif mode_kind == "showcase":
            resolved_bt_config = "Scripts/ConfigJson/showcase_competition.json"
        else:
            resolved_bt_config = "Scripts/ConfigJson/regional_competition.json"

        return [
            SetLaunchConfiguration("resolved_mode_kind", mode_kind),
            SetLaunchConfiguration("resolved_competition_profile", resolved_profile),
            SetLaunchConfiguration("resolved_bt_config_file", resolved_bt_config),
        ]

    # 分层配置默认入口：
    #   base + module + optional global override(config_file)
    behavior_tree_share = get_package_share_directory("behavior_tree")
    detector_share = get_package_share_directory("detector")
    predictor_share = get_package_share_directory("predictor")
    outpost_share = get_package_share_directory("outpost_hitter")
    buff_share = get_package_share_directory("buff_hitter")
    behavior_tree_config_root = os.path.join(behavior_tree_share, "config")
    default_base_config_file = os.path.join(behavior_tree_config_root, "base_config.yaml")
    default_override_config_file = os.path.join(behavior_tree_config_root, "override_config.yaml")
    default_detector_config_file = os.path.join(detector_share, "config", "detector_config.yaml")
    default_predictor_config_file = os.path.join(predictor_share, "config", "predictor_config.yaml")
    default_outpost_config_file = os.path.join(outpost_share, "config", "outpost_config.yaml")
    default_buff_config_file = os.path.join(buff_share, "config", "buff_config.yaml")

    mode = LaunchConfiguration("mode")
    config_file = LaunchConfiguration("config_file")
    base_config_file = LaunchConfiguration("base_config_file")
    detector_config_file = LaunchConfiguration("detector_config_file")
    predictor_config_file = LaunchConfiguration("predictor_config_file")
    outpost_config_file = LaunchConfiguration("outpost_config_file")
    buff_config_file = LaunchConfiguration("buff_config_file")
    output = LaunchConfiguration("output")
    competition_profile = LaunchConfiguration("competition_profile")
    bt_config_file = LaunchConfiguration("bt_config_file")
    bt_tree_file = LaunchConfiguration("bt_tree_file")
    debug_bypass_is_start = LaunchConfiguration("debug_bypass_is_start")
    runtime_rearm_start_gate = LaunchConfiguration("runtime_rearm_start_gate")
    publish_navi_goal = LaunchConfiguration("publish_navi_goal")
    wait_for_game_start_timeout_sec = LaunchConfiguration("wait_for_game_start_timeout_sec")
    league_referee_stale_timeout_ms = LaunchConfiguration("league_referee_stale_timeout_ms")
    decision_trace_enabled = LaunchConfiguration("decision_trace_enabled")
    decision_trace_file = LaunchConfiguration("decision_trace_file")
    decision_trace_every_n_ticks = LaunchConfiguration("decision_trace_every_n_ticks")

    use_gimbal = LaunchConfiguration("use_gimbal")
    use_detector = LaunchConfiguration("use_detector")
    use_tracker = LaunchConfiguration("use_tracker")
    use_predictor = LaunchConfiguration("use_predictor")
    use_outpost = LaunchConfiguration("use_outpost")
    use_buff = LaunchConfiguration("use_buff")
    use_behavior_tree = LaunchConfiguration("use_behavior_tree")
    offline = LaunchConfiguration("offline")
    resolved_mode_kind = LaunchConfiguration("resolved_mode_kind")
    resolved_competition_profile = LaunchConfiguration("resolved_competition_profile")
    resolved_bt_config_file = LaunchConfiguration("resolved_bt_config_file")

    launch_args = [
        DeclareLaunchArgument(
            "mode",
            default_value="",
            description="Startup mode: league/regional/showcase. Empty falls back to regional unless overridden.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_override_config_file,
            description="Optional global override YAML (applied last to all nodes).",
        ),
        DeclareLaunchArgument(
            "base_config_file",
            default_value=default_base_config_file,
            description="Base shared YAML for camera/solver/io.",
        ),
        DeclareLaunchArgument(
            "detector_config_file",
            default_value=default_detector_config_file,
            description="Detector module YAML.",
        ),
        DeclareLaunchArgument(
            "predictor_config_file",
            default_value=default_predictor_config_file,
            description="Predictor/tracker module YAML.",
        ),
        DeclareLaunchArgument(
            "outpost_config_file",
            default_value=default_outpost_config_file,
            description="Outpost module YAML.",
        ),
        DeclareLaunchArgument(
            "buff_config_file",
            default_value=default_buff_config_file,
            description="Buff module YAML.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument(
            "competition_profile",
            default_value="",
            description="behavior_tree profile override: regional or league. Empty keeps config/default.",
        ),
        DeclareLaunchArgument(
            "bt_config_file",
            default_value="",
            description="Optional behavior_tree JSON config path. Relative paths resolve under behavior_tree share dir.",
        ),
        DeclareLaunchArgument(
            "bt_tree_file",
            default_value="",
            description="Optional behavior_tree XML path. Relative paths resolve under behavior_tree share dir.",
        ),
        DeclareLaunchArgument(
            "debug_bypass_is_start",
            default_value="false",
            description="Debug only: true will bypass waiting /ly/game/is_start gate.",
        ),
        DeclareLaunchArgument(
            "runtime_rearm_start_gate",
            default_value="false",
            description="Debug only: when true, game loop re-enters start gate if /ly/game/is_start becomes false.",
        ),
        DeclareLaunchArgument(
            "publish_navi_goal",
            default_value="true",
            description="Whether behavior_tree publishes /ly/navi/goal and /ly/navi/goal_pos.",
        ),
        DeclareLaunchArgument(
            "wait_for_game_start_timeout_sec",
            default_value="0",
            description="0 disables timeout. >0 continues after waiting this many seconds for /ly/game/is_start.",
        ),
        DeclareLaunchArgument(
            "league_referee_stale_timeout_ms",
            default_value="0",
            description="0 disables stale-check. >0 enables league referee freshness guard for HP/Ammo recovery.",
        ),
        DeclareLaunchArgument(
            "decision_trace_enabled",
            default_value="false",
            description="Debug only: enable JSONL decision trace for offline pygame replay.",
        ),
        DeclareLaunchArgument(
            "decision_trace_file",
            default_value="",
            description="JSONL decision trace output path. Used only when decision_trace_enabled is true.",
        ),
        DeclareLaunchArgument(
            "decision_trace_every_n_ticks",
            default_value="5",
            description="Write one decision trace record every N behavior_tree ticks when tracing is enabled.",
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_detector", default_value="true"),
        DeclareLaunchArgument("use_tracker", default_value="true"),
        DeclareLaunchArgument("use_predictor", default_value="true"),
        DeclareLaunchArgument("use_outpost", default_value="true"),
        DeclareLaunchArgument("use_buff", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument(
            "offline",
            default_value="false",
            description="Offline profile: force virtual IO and video replay without editing YAML.",
        ),
        DeclareLaunchArgument("resolved_mode_kind", default_value=""),
        DeclareLaunchArgument("resolved_competition_profile", default_value=""),
        DeclareLaunchArgument("resolved_bt_config_file", default_value=""),
        OpaqueFunction(function=resolve_mode_defaults),
    ]

    info_logs = [
        LogInfo(msg=["[sentry_all] mode: ", mode]),
        LogInfo(msg=["[sentry_all] config: ", config_file]),
        LogInfo(msg=["[sentry_all] base_config: ", base_config_file]),
        LogInfo(msg=["[sentry_all] detector_config: ", detector_config_file]),
        LogInfo(msg=["[sentry_all] predictor_config: ", predictor_config_file]),
        LogInfo(msg=["[sentry_all] outpost_config: ", outpost_config_file]),
        LogInfo(msg=["[sentry_all] buff_config: ", buff_config_file]),
        LogInfo(msg=["[sentry_all] output: ", output]),
        LogInfo(msg=["[sentry_all] offline: ", offline]),
        LogInfo(msg=["[sentry_all] competition_profile: ", competition_profile]),
        LogInfo(msg=["[sentry_all] bt_config_file: ", bt_config_file]),
        LogInfo(msg=["[sentry_all] resolved_mode: ", resolved_mode_kind]),
        LogInfo(msg=["[sentry_all] resolved_competition_profile: ", resolved_competition_profile]),
        LogInfo(msg=["[sentry_all] resolved_bt_config_file: ", resolved_bt_config_file]),
        LogInfo(msg=["[sentry_all] bt_tree_file: ", bt_tree_file]),
        LogInfo(msg=["[sentry_all] debug_bypass_is_start: ", debug_bypass_is_start]),
        LogInfo(msg=["[sentry_all] runtime_rearm_start_gate: ", runtime_rearm_start_gate]),
        LogInfo(msg=["[sentry_all] publish_navi_goal: ", publish_navi_goal]),
        LogInfo(msg=["[sentry_all] wait_for_game_start_timeout_sec: ", wait_for_game_start_timeout_sec]),
        LogInfo(msg=["[sentry_all] league_referee_stale_timeout_ms: ", league_referee_stale_timeout_ms]),
        LogInfo(msg=["[sentry_all] decision_trace_enabled: ", decision_trace_enabled]),
        LogInfo(msg=["[sentry_all] decision_trace_file: ", decision_trace_file]),
        LogInfo(msg=["[sentry_all] decision_trace_every_n_ticks: ", decision_trace_every_n_ticks]),
    ]

    nodes = [
        # gimbal_driver: offline=true 时强制 use_virtual_device
        GroupAction(
            actions=[
                Node(
                    package="gimbal_driver",
                    executable="gimbal_driver_node",
                    name="gimbal_driver",
                    output=output,
                    parameters=[base_config_file, config_file],
                    on_exit=Shutdown(reason="gimbal_driver exited"),
                    condition=LaunchConfigurationNotEquals("offline", "true"),
                ),
                Node(
                    package="gimbal_driver",
                    executable="gimbal_driver_node",
                    name="gimbal_driver",
                    output=output,
                    parameters=[
                        base_config_file,
                        config_file,
                        {
                            "io_config/use_virtual_device": True,
                            "io_config.use_virtual_device": True,
                        },
                    ],
                    on_exit=Shutdown(reason="gimbal_driver exited"),
                    condition=LaunchConfigurationEquals("offline", "true"),
                ),
            ],
            condition=IfCondition(use_gimbal),
        ),
        # detector: offline=true 时强制 use_video
        GroupAction(
            actions=[
                Node(
                    package="detector",
                    executable="detector_node",
                    name="detector",
                    output=output,
                    parameters=[base_config_file, detector_config_file, config_file],
                    on_exit=Shutdown(reason="detector exited"),
                    condition=LaunchConfigurationNotEquals("offline", "true"),
                ),
                Node(
                    package="detector",
                    executable="detector_node",
                    name="detector",
                    output=output,
                    parameters=[
                        base_config_file,
                        detector_config_file,
                        config_file,
                        {
                            "detector_config/use_video": True,
                            "detector_config.use_video": True,
                        },
                    ],
                    on_exit=Shutdown(reason="detector exited"),
                    condition=LaunchConfigurationEquals("offline", "true"),
                ),
            ],
            condition=IfCondition(use_detector),
        ),
        # 下游链路节点
        Node(
            package="tracker_solver",
            executable="tracker_solver_node",
            name="tracker_solver",
            output=output,
            parameters=[base_config_file, predictor_config_file, config_file],
            on_exit=Shutdown(reason="tracker_solver exited"),
            condition=IfCondition(use_tracker),
        ),
        Node(
            package="predictor",
            executable="predictor_node",
            name="predictor_node",
            output=output,
            parameters=[base_config_file, predictor_config_file, config_file],
            on_exit=Shutdown(reason="predictor_node exited"),
            condition=IfCondition(use_predictor),
        ),
        Node(
            package="outpost_hitter",
            executable="outpost_hitter_node",
            name="outpost_hitter",
            output=output,
            parameters=[base_config_file, outpost_config_file, config_file],
            on_exit=Shutdown(reason="outpost_hitter exited"),
            condition=IfCondition(use_outpost),
        ),
        Node(
            package="buff_hitter",
            executable="buff_hitter_node",
            name="buff_hitter",
            output=output,
            parameters=[base_config_file, buff_config_file, config_file],
            on_exit=Shutdown(reason="buff_hitter exited"),
            condition=IfCondition(use_buff),
        ),
        # 最后启动 behavior_tree（决策接管）
        Node(
            package="behavior_tree",
            executable="behavior_tree_node",
            name="behavior_tree",
            output=output,
            parameters=[
                {
                    "competition_profile": resolved_competition_profile,
                    "bt_config_file": resolved_bt_config_file,
                    "bt_tree_file": bt_tree_file,
                    "debug_bypass_is_start": debug_bypass_is_start,
                    "runtime_rearm_start_gate": runtime_rearm_start_gate,
                    "publish_navi_goal": publish_navi_goal,
                    "wait_for_game_start_timeout_sec": wait_for_game_start_timeout_sec,
                    "league_referee_stale_timeout_ms": league_referee_stale_timeout_ms,
                    "decision_trace_enabled": decision_trace_enabled,
                    "decision_trace_file": decision_trace_file,
                    "decision_trace_every_n_ticks": decision_trace_every_n_ticks,
                }
            ],
            on_exit=Shutdown(reason="behavior_tree exited"),
            condition=IfCondition(use_behavior_tree),
        ),
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
