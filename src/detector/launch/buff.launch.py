#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
打符链路启动入口。

默认启动：
- gimbal_driver
- buff_hitter

说明：
- 本 launch 仅覆盖打符相关链路，不包含 detector/tracker/predictor。
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
        default_config_file = os.path.join(
            behavior_tree_share, "config", "auto_aim_config_competition.yaml"
        )
    except Exception:
        detector_share = get_package_share_directory("detector")
        default_config_file = os.path.join(detector_share, "config", "auto_aim_config.yaml")

    config_file = LaunchConfiguration("config_file")
    output = LaunchConfiguration("output")
    use_gimbal = LaunchConfiguration("use_gimbal")
    use_buff = LaunchConfiguration("use_buff")

    launch_args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Shared YAML config file for buff chain.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_buff", default_value="true"),
    ]

    info_logs = [
        LogInfo(msg=["[buff] config: ", config_file]),
        LogInfo(msg=["[buff] output: ", output]),
    ]

    nodes = [
        Node(
            package="gimbal_driver",
            executable="gimbal_driver_node",
            name="gimbal_driver",
            output=output,
            parameters=[config_file],
            on_exit=Shutdown(reason="gimbal_driver exited"),
            condition=IfCondition(use_gimbal),
        ),
        Node(
            package="buff_hitter",
            executable="buff_hitter_node",
            name="buff_hitter",
            output=output,
            parameters=[config_file],
            on_exit=Shutdown(reason="buff_hitter exited"),
            condition=IfCondition(use_buff),
        ),
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
