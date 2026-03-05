#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    use_detector = LaunchConfiguration("use_detector")
    use_tracker = LaunchConfiguration("use_tracker")
    use_predictor = LaunchConfiguration("use_predictor")
    use_outpost = LaunchConfiguration("use_outpost")
    use_buff = LaunchConfiguration("use_buff")
    use_behavior_tree = LaunchConfiguration("use_behavior_tree")

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
    ]

    nodes = [
        Node(
            package="gimbal_driver",
            executable="gimbal_driver_node",
            name="gimbal_driver",
            output=output,
            parameters=[config_file],
            condition=IfCondition(use_gimbal),
        ),
        Node(
            package="detector",
            executable="detector_node",
            name="detector",
            output=output,
            parameters=[config_file],
            condition=IfCondition(use_detector),
        ),
        Node(
            package="tracker_solver",
            executable="tracker_solver_node",
            name="tracker_solver",
            output=output,
            parameters=[config_file],
            condition=IfCondition(use_tracker),
        ),
        Node(
            package="predictor",
            executable="predictor_node",
            name="predictor_node",
            output=output,
            parameters=[config_file],
            condition=IfCondition(use_predictor),
        ),
        Node(
            package="outpost_hitter",
            executable="outpost_hitter_node",
            name="outpost_hitter",
            output=output,
            parameters=[config_file],
            condition=IfCondition(use_outpost),
        ),
        Node(
            package="buff_hitter",
            executable="buff_hitter_node",
            name="buff_hitter",
            output=output,
            parameters=[config_file],
            condition=IfCondition(use_buff),
        ),
        Node(
            package="behavior_tree",
            executable="behavior_tree_node",
            name="behavior_tree",
            output=output,
            emulate_tty=True,
            condition=IfCondition(use_behavior_tree),
        ),
    ]

    return LaunchDescription(launch_args + nodes)
