#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    behavior_tree_share = get_package_share_directory("behavior_tree")
    sentry_all_launch = os.path.join(behavior_tree_share, "launch", "sentry_all.launch.py")
    default_config_file = os.path.join(behavior_tree_share, "config", "auto_aim_config_competition.yaml")

    launch_args = [
        DeclareLaunchArgument("config_file", default_value=default_config_file),
        DeclareLaunchArgument("output", default_value="screen"),
        DeclareLaunchArgument("offline", default_value="false"),
        DeclareLaunchArgument("debug_bypass_is_start", default_value="true"),
        DeclareLaunchArgument("wait_for_game_start_timeout_sec", default_value="0"),
        DeclareLaunchArgument("league_referee_stale_timeout_ms", default_value="0"),
        DeclareLaunchArgument("bt_tree_file", default_value=""),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_detector", default_value="true"),
        DeclareLaunchArgument("use_tracker", default_value="true"),
        DeclareLaunchArgument("use_predictor", default_value="true"),
        DeclareLaunchArgument("use_outpost", default_value="true"),
        DeclareLaunchArgument("use_buff", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
    ]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sentry_all_launch),
        launch_arguments={
            "mode": "showcase",
            "config_file": LaunchConfiguration("config_file"),
            "output": LaunchConfiguration("output"),
            "offline": LaunchConfiguration("offline"),
            "debug_bypass_is_start": LaunchConfiguration("debug_bypass_is_start"),
            "wait_for_game_start_timeout_sec": LaunchConfiguration("wait_for_game_start_timeout_sec"),
            "league_referee_stale_timeout_ms": LaunchConfiguration("league_referee_stale_timeout_ms"),
            "bt_tree_file": LaunchConfiguration("bt_tree_file"),
            "use_gimbal": LaunchConfiguration("use_gimbal"),
            "use_detector": LaunchConfiguration("use_detector"),
            "use_tracker": LaunchConfiguration("use_tracker"),
            "use_predictor": LaunchConfiguration("use_predictor"),
            "use_outpost": LaunchConfiguration("use_outpost"),
            "use_buff": LaunchConfiguration("use_buff"),
            "use_behavior_tree": LaunchConfiguration("use_behavior_tree"),
        }.items(),
    )

    return LaunchDescription(launch_args + [include])
