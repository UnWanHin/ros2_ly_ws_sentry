#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
射表标定链路启动入口。

默认启动：
- gimbal_driver
- shooting_table_calib_node

说明：
- 会额外向标定节点注入 detector 模型路径和可视化参数。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    detector_share = get_package_share_directory("detector")
    behavior_tree_share = get_package_share_directory("behavior_tree")
    config_root = os.path.join(behavior_tree_share, "config", "stack")
    default_base_config_file = os.path.join(config_root, "base_competition.yaml")
    default_detector_config_file = os.path.join(config_root, "detector_competition.yaml")
    default_predictor_config_file = os.path.join(config_root, "predictor_competition.yaml")
    default_override_config_file = os.path.join(config_root, "override_none.yaml")

    config_file = LaunchConfiguration("config_file")
    base_config_file = LaunchConfiguration("base_config_file")
    detector_config_file = LaunchConfiguration("detector_config_file")
    predictor_config_file = LaunchConfiguration("predictor_config_file")
    output = LaunchConfiguration("output")
    use_gimbal = LaunchConfiguration("use_gimbal")
    use_calib = LaunchConfiguration("use_calib")
    team_red = LaunchConfiguration("team_red")
    debug_team_blue = LaunchConfiguration("debug_team_blue")
    web_show = LaunchConfiguration("web_show")
    draw_image = LaunchConfiguration("draw_image")
    auto_lock_fire = LaunchConfiguration("auto_lock_fire")
    auto_fire = LaunchConfiguration("auto_fire")
    auto_target_type = LaunchConfiguration("auto_target_type")
    record_dir = LaunchConfiguration("record_dir")
    csv_strategy = LaunchConfiguration("csv_strategy")
    csv_path = LaunchConfiguration("csv_path")

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
            "detector_config_file",
            default_value=default_detector_config_file,
            description="Detector module YAML.",
        ),
        DeclareLaunchArgument(
            "predictor_config_file",
            default_value=default_predictor_config_file,
            description="Predictor/controller module YAML.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_calib", default_value="true"),
        DeclareLaunchArgument("team_red", default_value="true"),
        DeclareLaunchArgument("debug_team_blue", default_value="false"),
        DeclareLaunchArgument("web_show", default_value="true"),
        DeclareLaunchArgument("draw_image", default_value="true"),
        DeclareLaunchArgument("auto_lock_fire", default_value="false"),
        DeclareLaunchArgument("auto_fire", default_value="true"),
        DeclareLaunchArgument("auto_target_type", default_value="4"),
        DeclareLaunchArgument("record_dir", default_value=""),
        DeclareLaunchArgument("csv_strategy", default_value="new"),
        DeclareLaunchArgument("csv_path", default_value=""),
    ]

    info_logs = [
        LogInfo(msg=["[shooting_table_calib] config: ", config_file]),
        LogInfo(msg=["[shooting_table_calib] base_config: ", base_config_file]),
        LogInfo(msg=["[shooting_table_calib] detector_config: ", detector_config_file]),
        LogInfo(msg=["[shooting_table_calib] predictor_config: ", predictor_config_file]),
        LogInfo(msg=["[shooting_table_calib] output: ", output]),
        LogInfo(msg=["[shooting_table_calib] auto_lock_fire: ", auto_lock_fire]),
        LogInfo(msg=["[shooting_table_calib] auto_fire: ", auto_fire]),
        LogInfo(msg=["[shooting_table_calib] auto_target_type: ", auto_target_type]),
        LogInfo(msg=["[shooting_table_calib] record_dir: ", record_dir]),
        LogInfo(msg=["[shooting_table_calib] csv_strategy: ", csv_strategy]),
        LogInfo(msg=["[shooting_table_calib] csv_path: ", csv_path]),
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
            package="shooting_table_calib",
            executable="shooting_table_calib_node",
            name="shooting_table_calib",
            output=output,
            parameters=[
                base_config_file,
                detector_config_file,
                predictor_config_file,
                config_file,
                {
                    "detector_config.classifier_path": os.path.join(detector_share, "Extras", "classifier.xml"),
                    "detector_config.detector_path": os.path.join(detector_share, "Extras", "armor_detector_model.xml"),
                    "detector_config.car_model_path": os.path.join(detector_share, "Extras", "car_detector_model.xml"),
                    "team_red": team_red,
                    "detector_config.debug_team_blue": debug_team_blue,
                    "detector_config/debug_team_blue": debug_team_blue,
                    "web_show": web_show,
                    "draw_image": draw_image,
                    "shooting_table_calib.auto_lock_fire": auto_lock_fire,
                    "shooting_table_calib.auto_fire": auto_fire,
                    "shooting_table_calib.auto_target_type": auto_target_type,
                    "shooting_table_calib.record_dir": record_dir,
                    "shooting_table_calib.csv_strategy": csv_strategy,
                    "shooting_table_calib.csv_path": csv_path,
                }
            ],
            on_exit=Shutdown(reason="shooting_table_calib exited"),
            condition=IfCondition(use_calib),
        ),
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
