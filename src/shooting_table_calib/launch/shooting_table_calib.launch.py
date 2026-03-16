#!/usr/bin/env python3
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
    default_config_file = os.path.join(detector_share, "config", "auto_aim_config.yaml")

    config_file = LaunchConfiguration("config_file")
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

    launch_args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Shared YAML config file for calibration chain.",
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
    ]

    info_logs = [
        LogInfo(msg=["[shooting_table_calib] config: ", config_file]),
        LogInfo(msg=["[shooting_table_calib] output: ", output]),
        LogInfo(msg=["[shooting_table_calib] auto_lock_fire: ", auto_lock_fire]),
        LogInfo(msg=["[shooting_table_calib] auto_fire: ", auto_fire]),
        LogInfo(msg=["[shooting_table_calib] auto_target_type: ", auto_target_type]),
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
            package="shooting_table_calib",
            executable="shooting_table_calib_node",
            name="shooting_table_calib",
            output=output,
            parameters=[
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
                }
            ],
            on_exit=Shutdown(reason="shooting_table_calib exited"),
            condition=IfCondition(use_calib),
        ),
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
