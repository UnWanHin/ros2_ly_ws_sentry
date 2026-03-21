#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
behavior_tree 单包启动入口。

用途：
- 仅启动 behavior_tree_node，适合独立调试决策流程。
- 不负责拉起 detector / gimbal_driver 等下游节点。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output = LaunchConfiguration("output")
    competition_profile = LaunchConfiguration("competition_profile")
    bt_config_file = LaunchConfiguration("bt_config_file")
    bt_tree_file = LaunchConfiguration("bt_tree_file")
    debug_bypass_is_start = LaunchConfiguration("debug_bypass_is_start")
    wait_for_game_start_timeout_sec = LaunchConfiguration("wait_for_game_start_timeout_sec")
    league_referee_stale_timeout_ms = LaunchConfiguration("league_referee_stale_timeout_ms")

    launch_args = [
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
            "wait_for_game_start_timeout_sec",
            default_value="0",
            description="0 disables timeout. >0 continues after waiting this many seconds for /ly/game/is_start.",
        ),
        DeclareLaunchArgument(
            "league_referee_stale_timeout_ms",
            default_value="0",
            description="0 disables stale-check. >0 enables league referee freshness guard for HP/Ammo recovery.",
        ),
    ]

    info_logs = [
        LogInfo(msg=["[behavior_tree] output: ", output]),
        LogInfo(msg=["[behavior_tree] competition_profile: ", competition_profile]),
        LogInfo(msg=["[behavior_tree] bt_config_file: ", bt_config_file]),
        LogInfo(msg=["[behavior_tree] bt_tree_file: ", bt_tree_file]),
        LogInfo(msg=["[behavior_tree] debug_bypass_is_start: ", debug_bypass_is_start]),
        LogInfo(msg=["[behavior_tree] wait_for_game_start_timeout_sec: ", wait_for_game_start_timeout_sec]),
        LogInfo(msg=["[behavior_tree] league_referee_stale_timeout_ms: ", league_referee_stale_timeout_ms]),
    ]

    nodes = [
        Node(
            package="behavior_tree",
            executable="behavior_tree_node",
            name="behavior_tree",
            output=output,
            parameters=[
                {
                    "competition_profile": competition_profile,
                    "bt_config_file": bt_config_file,
                    "bt_tree_file": bt_tree_file,
                    "debug_bypass_is_start": debug_bypass_is_start,
                    "wait_for_game_start_timeout_sec": wait_for_game_start_timeout_sec,
                    "league_referee_stale_timeout_ms": league_referee_stale_timeout_ms,
                }
            ],
            on_exit=Shutdown(reason="behavior_tree exited"),
        )
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
