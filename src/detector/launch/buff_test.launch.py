#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
纯打符联调入口（不启动 behavior_tree，不做巡逻/策略导航）。

启动节点：
- gimbal_driver
- detector
- buff_hitter
- buff_test_bridge（固定 aa=false/ra=true，并将 /ly/buff/target 桥接到 /ly/control/*）
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    try:
        behavior_tree_share = get_package_share_directory("behavior_tree")
        detector_share = get_package_share_directory("detector")
        buff_share = get_package_share_directory("buff_hitter")
        config_root = os.path.join(behavior_tree_share, "config")
        default_base_config_file = os.path.join(config_root, "base_config.yaml")
        default_override_config_file = os.path.join(config_root, "override_config.yaml")
        default_detector_config_file = os.path.join(detector_share, "config", "detector_config.yaml")
        default_buff_config_file = os.path.join(buff_share, "config", "buff_config.yaml")
    except Exception:
        default_base_config_file = "config/base_config.yaml"
        default_override_config_file = "config/override_config.yaml"
        default_detector_config_file = "src/detector/config/detector_config.yaml"
        default_buff_config_file = "src/buff_hitter/config/buff_config.yaml"

    config_file = LaunchConfiguration("config_file")
    base_config_file = LaunchConfiguration("base_config_file")
    detector_config_file = LaunchConfiguration("detector_config_file")
    buff_config_file = LaunchConfiguration("buff_config_file")
    output = LaunchConfiguration("output")

    use_gimbal = LaunchConfiguration("use_gimbal")
    use_detector = LaunchConfiguration("use_detector")
    use_buff = LaunchConfiguration("use_buff")
    use_bridge = LaunchConfiguration("use_bridge")

    bridge_target_topic = LaunchConfiguration("bridge_target_topic")
    bridge_enable_fire = LaunchConfiguration("bridge_enable_fire")
    bridge_fire_hz = LaunchConfiguration("bridge_fire_hz")
    bridge_publish_hz = LaunchConfiguration("bridge_publish_hz")
    bridge_gate_hz = LaunchConfiguration("bridge_gate_hz")
    bridge_timeout_sec = LaunchConfiguration("bridge_timeout_sec")
    bridge_zero_velocity = LaunchConfiguration("bridge_zero_velocity")

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
            "buff_config_file",
            default_value=default_buff_config_file,
            description="Buff module YAML.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_detector", default_value="true"),
        DeclareLaunchArgument("use_buff", default_value="true"),
        DeclareLaunchArgument("use_bridge", default_value="true"),
        DeclareLaunchArgument("bridge_target_topic", default_value="/ly/buff/target"),
        DeclareLaunchArgument("bridge_enable_fire", default_value="true"),
        DeclareLaunchArgument("bridge_fire_hz", default_value="20.0"),
        DeclareLaunchArgument("bridge_publish_hz", default_value="80.0"),
        DeclareLaunchArgument("bridge_gate_hz", default_value="2.0"),
        DeclareLaunchArgument("bridge_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("bridge_zero_velocity", default_value="true"),
    ]

    info_logs = [
        LogInfo(msg=["[buff_test] config: ", config_file]),
        LogInfo(msg=["[buff_test] base_config: ", base_config_file]),
        LogInfo(msg=["[buff_test] detector_config: ", detector_config_file]),
        LogInfo(msg=["[buff_test] buff_config: ", buff_config_file]),
        LogInfo(msg=["[buff_test] output: ", output]),
        LogInfo(msg=["[buff_test] bridge_target_topic: ", bridge_target_topic]),
        LogInfo(msg=["[buff_test] bridge_enable_fire: ", bridge_enable_fire]),
        LogInfo(msg=["[buff_test] bridge_fire_hz: ", bridge_fire_hz]),
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
            package="detector",
            executable="detector_node",
            name="detector",
            output=output,
            parameters=[base_config_file, detector_config_file, config_file],
            on_exit=Shutdown(reason="detector exited"),
            condition=IfCondition(use_detector),
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
        Node(
            package="detector",
            executable="buff_test_bridge",
            name="buff_test_bridge",
            output=output,
            parameters=[{
                "target_topic": ParameterValue(bridge_target_topic, value_type=str),
                "enable_fire": ParameterValue(bridge_enable_fire, value_type=bool),
                "fire_hz": ParameterValue(bridge_fire_hz, value_type=float),
                "publish_hz": ParameterValue(bridge_publish_hz, value_type=float),
                "gate_hz": ParameterValue(bridge_gate_hz, value_type=float),
                "timeout_sec": ParameterValue(bridge_timeout_sec, value_type=float),
                "zero_velocity": ParameterValue(bridge_zero_velocity, value_type=bool),
            }],
            on_exit=Shutdown(reason="buff_test_bridge exited"),
            condition=IfCondition(use_bridge),
        ),
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
