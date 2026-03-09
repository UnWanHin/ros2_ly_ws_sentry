#!/usr/bin/env python3
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
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, Shutdown
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # detector 包内统一参数文件（被多个节点共享）
    detector_share = get_package_share_directory("detector")
    default_config_file = os.path.join(detector_share, "config", "auto_aim_config.yaml")

    config_file = LaunchConfiguration("config_file")
    output = LaunchConfiguration("output")

    use_gimbal = LaunchConfiguration("use_gimbal")
    use_detector = LaunchConfiguration("use_detector")
    use_tracker = LaunchConfiguration("use_tracker")
    use_predictor = LaunchConfiguration("use_predictor")
    use_outpost = LaunchConfiguration("use_outpost")
    use_buff = LaunchConfiguration("use_buff")
    use_behavior_tree = LaunchConfiguration("use_behavior_tree")
    offline = LaunchConfiguration("offline")

    launch_args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Shared YAML config file for perception/solver/driver nodes.",
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
        DeclareLaunchArgument("use_outpost", default_value="true"),
        DeclareLaunchArgument("use_buff", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument(
            "offline",
            default_value="false",
            description="Offline profile: force virtual IO and video replay without editing YAML.",
        ),
    ]

    info_logs = [
        LogInfo(msg=["[sentry_all] config: ", config_file]),
        LogInfo(msg=["[sentry_all] output: ", output]),
        LogInfo(msg=["[sentry_all] offline: ", offline]),
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
        # detector: offline=true 时强制 use_video
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
        # 下游链路节点
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
        Node(
            package="outpost_hitter",
            executable="outpost_hitter_node",
            name="outpost_hitter",
            output=output,
            parameters=[config_file],
            on_exit=Shutdown(reason="outpost_hitter exited"),
            condition=IfCondition(use_outpost),
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
        # 最后启动 behavior_tree（决策接管）
        Node(
            package="behavior_tree",
            executable="behavior_tree_node",
            name="behavior_tree",
            output=output,
            on_exit=Shutdown(reason="behavior_tree exited"),
            condition=IfCondition(use_behavior_tree),
        ),
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
