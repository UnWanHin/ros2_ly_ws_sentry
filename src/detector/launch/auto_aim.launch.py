#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
自瞄链路启动入口（不含 behavior_tree）。

默认链路：
- gimbal_driver + detector + tracker_solver + predictor

可选：
- use_mapper=true 时附加 mapper_node（用于 BT 外火控联调）

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
    # Prefer the competition config used by the full sentry stack so detector /
    # tracker / predictor / behavior_tree stay on one YAML by default.
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
    use_detector = LaunchConfiguration("use_detector")
    use_tracker = LaunchConfiguration("use_tracker")
    use_predictor = LaunchConfiguration("use_predictor")
    use_mapper = LaunchConfiguration("use_mapper")
    offline = LaunchConfiguration("offline")

    mapper_red = LaunchConfiguration("mapper_red")
    mapper_target_id = LaunchConfiguration("mapper_target_id")
    mapper_enable_fire = LaunchConfiguration("mapper_enable_fire")
    mapper_auto_fire = LaunchConfiguration("mapper_auto_fire")
    mapper_diag_period = LaunchConfiguration("mapper_diag_period")
    mapper_angles_topic = LaunchConfiguration("mapper_angles_topic")
    mapper_firecode_topic = LaunchConfiguration("mapper_firecode_topic")

    launch_args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Shared YAML config file for detector pipeline nodes.",
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
        DeclareLaunchArgument("use_mapper", default_value="false"),
        DeclareLaunchArgument(
            "offline",
            default_value="false",
            description="Offline profile: force virtual IO and video replay without editing YAML.",
        ),
        DeclareLaunchArgument("mapper_red", default_value="true"),
        DeclareLaunchArgument("mapper_target_id", default_value="6"),
        DeclareLaunchArgument("mapper_enable_fire", default_value="true"),
        DeclareLaunchArgument("mapper_auto_fire", default_value="true"),
        DeclareLaunchArgument("mapper_diag_period", default_value="1.0"),
        DeclareLaunchArgument(
            "mapper_angles_topic",
            default_value="/ly/debug/control/angles",
            description="Remap target for mapper /ly/control/angles output. Use /ly/control/angles to drive gimbal.",
        ),
        DeclareLaunchArgument(
            "mapper_firecode_topic",
            default_value="/ly/debug/control/firecode",
            description="Remap target for mapper /ly/control/firecode output. Use /ly/control/firecode to drive gimbal.",
        ),
    ]

    info_logs = [
        LogInfo(msg=["[auto_aim] config: ", config_file]),
        LogInfo(msg=["[auto_aim] output: ", output]),
        LogInfo(msg=["[auto_aim] offline: ", offline]),
        LogInfo(msg=["[auto_aim] use_mapper: ", use_mapper]),
        LogInfo(msg=["[auto_aim] mapper angles -> ", mapper_angles_topic]),
        LogInfo(msg=["[auto_aim] mapper firecode -> ", mapper_firecode_topic]),
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
                    parameters=[config_file],
                    on_exit=Shutdown(reason="gimbal_driver exited"),
                    condition=LaunchConfigurationNotEquals("offline", "true"),
                ),
                Node(
                    package="gimbal_driver",
                    executable="gimbal_driver_node",
                    name="gimbal_driver",
                    output=output,
                    parameters=[
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
                    parameters=[config_file],
                    on_exit=Shutdown(reason="detector exited"),
                    condition=LaunchConfigurationNotEquals("offline", "true"),
                ),
                Node(
                    package="detector",
                    executable="detector_node",
                    name="detector",
                    output=output,
                    parameters=[
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
            parameters=[config_file],
            on_exit=Shutdown(reason="tracker_solver exited"),
            condition=IfCondition(use_tracker),
        ),
        Node(
            package="predictor",
            executable="predictor_node",
            name="predictor_node",
            output=output,
            parameters=[config_file],
            on_exit=Shutdown(reason="predictor_node exited"),
            condition=IfCondition(use_predictor),
        ),
        # 可选 mapper（默认输出到 debug 话题，避免抢 /ly/control/*）
        Node(
            package="detector",
            executable="mapper_node",
            name="target_to_gimbal_mapper",
            output=output,
            arguments=[
                "--red",
                mapper_red,
                "--target-id",
                mapper_target_id,
                "--enable-fire",
                mapper_enable_fire,
                "--auto-fire",
                mapper_auto_fire,
                "--diag-period",
                mapper_diag_period,
            ],
            remappings=[
                ("/ly/control/angles", mapper_angles_topic),
                ("/ly/control/firecode", mapper_firecode_topic),
            ],
            on_exit=Shutdown(reason="mapper_node exited"),
            condition=IfCondition(use_mapper),
        ),
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
