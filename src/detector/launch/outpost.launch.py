#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
打前哨链路启动入口。

默认启动：
- gimbal_driver
- outpost_hitter

说明：
- 本 launch 仅覆盖前哨链路，不包含 detector/tracker/predictor。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    try:
        behavior_tree_share = get_package_share_directory("behavior_tree")
        outpost_share = get_package_share_directory("outpost_hitter")
        config_root = os.path.join(behavior_tree_share, "config")
        default_base_config_file = os.path.join(config_root, "base_config.yaml")
        default_override_config_file = os.path.join(config_root, "override_config.yaml")
        default_outpost_config_file = os.path.join(outpost_share, "config", "outpost_config.yaml")
    except Exception:
        default_base_config_file = "config/base_config.yaml"
        default_outpost_config_file = "src/outpost_hitter/config/outpost_config.yaml"
        default_override_config_file = "config/override_config.yaml"

    config_file = LaunchConfiguration("config_file")
    base_config_file = LaunchConfiguration("base_config_file")
    outpost_config_file = LaunchConfiguration("outpost_config_file")
    output = LaunchConfiguration("output")
    use_gimbal = LaunchConfiguration("use_gimbal")
    use_outpost = LaunchConfiguration("use_outpost")

    launch_args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_override_config_file,
            description="Optional global override YAML (applied last).",
        ),
        DeclareLaunchArgument(
            "base_config_file",
            default_value=default_base_config_file,
            description="Base shared YAML for camera/solver/io.",
        ),
        DeclareLaunchArgument(
            "outpost_config_file",
            default_value=default_outpost_config_file,
            description="Outpost module YAML.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_outpost", default_value="true"),
    ]

    info_logs = [
        LogInfo(msg=["[outpost] config: ", config_file]),
        LogInfo(msg=["[outpost] base_config: ", base_config_file]),
        LogInfo(msg=["[outpost] outpost_config: ", outpost_config_file]),
        LogInfo(msg=["[outpost] output: ", output]),
    ]

    nodes = [
        Node(
            package="gimbal_driver",
            executable="gimbal_driver_node",
            name="gimbal_driver",
            output=output,
            parameters=[base_config_file, config_file],
            on_exit=Shutdown(reason="gimbal_driver exited"),
            condition=IfCondition(use_gimbal),
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
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
