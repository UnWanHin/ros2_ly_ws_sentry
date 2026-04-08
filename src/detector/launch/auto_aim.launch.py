#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
自瞄链路启动入口（不含 behavior_tree）。

默认链路：
- gimbal_driver + detector + tracker_solver + predictor

离线模式：
- offline=true 会强制 use_virtual_device/use_video 覆盖，便于离车调试。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, Shutdown
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 分层配置：base + module + optional global override(config_file)
    try:
        behavior_tree_share = get_package_share_directory("behavior_tree")
        detector_share = get_package_share_directory("detector")
        predictor_share = get_package_share_directory("predictor")
        config_root = os.path.join(behavior_tree_share, "config")
        default_base_config_file = os.path.join(config_root, "base_config.yaml")
        default_override_config_file = os.path.join(config_root, "override_config.yaml")
        default_detector_config_file = os.path.join(detector_share, "config", "detector_config.yaml")
        default_predictor_config_file = os.path.join(predictor_share, "config", "predictor_config.yaml")
    except Exception:
        default_base_config_file = "config/base_config.yaml"
        default_detector_config_file = "src/detector/config/detector_config.yaml"
        default_predictor_config_file = "src/predictor/config/predictor_config.yaml"
        default_override_config_file = "config/override_config.yaml"

    config_file = LaunchConfiguration("config_file")
    base_config_file = LaunchConfiguration("base_config_file")
    detector_config_file = LaunchConfiguration("detector_config_file")
    predictor_config_file = LaunchConfiguration("predictor_config_file")
    output = LaunchConfiguration("output")

    use_gimbal = LaunchConfiguration("use_gimbal")
    use_detector = LaunchConfiguration("use_detector")
    use_tracker = LaunchConfiguration("use_tracker")
    use_predictor = LaunchConfiguration("use_predictor")
    offline = LaunchConfiguration("offline")

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
            description="Predictor/tracker module YAML.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_detector", default_value="true"),
        DeclareLaunchArgument("use_tracker", default_value="true"),
        DeclareLaunchArgument("use_predictor", default_value="true"),
        DeclareLaunchArgument(
            "offline",
            default_value="false",
            description="Offline profile: force virtual IO and video replay without editing YAML.",
        ),
    ]

    info_logs = [
        LogInfo(msg=["[auto_aim] config: ", config_file]),
        LogInfo(msg=["[auto_aim] base_config: ", base_config_file]),
        LogInfo(msg=["[auto_aim] detector_config: ", detector_config_file]),
        LogInfo(msg=["[auto_aim] predictor_config: ", predictor_config_file]),
        LogInfo(msg=["[auto_aim] output: ", output]),
        LogInfo(msg=["[auto_aim] offline: ", offline]),
    ]

    nodes = [
        # gimbal_driver: offline=true 时强制虚拟设备
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
        # detector: offline=true 时强制视频输入
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
        # 跟踪与预测
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
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
