#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bridge_share = get_package_share_directory("navi_tf_bridge")
    bridge_launch = os.path.join(bridge_share, "launch", "target_rel_to_goal_pos.launch.py")

    launch_args = [
        DeclareLaunchArgument("input_goal_pos_raw_topic", default_value="/ly/navi/goal_pos_raw"),
        DeclareLaunchArgument("output_goal_pos_topic", default_value="/ly/navi/goal_pos"),
        DeclareLaunchArgument("goal_pos_raw_frame", default_value="map"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("fallback_base_frame", default_value="baselink"),
        DeclareLaunchArgument("enable_goal_pos_raw_bridge", default_value="true"),
        DeclareLaunchArgument("debug_export_point_pairs", default_value="false"),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={
            "input_goal_pos_raw_topic": LaunchConfiguration("input_goal_pos_raw_topic"),
            "output_goal_pos_topic": LaunchConfiguration("output_goal_pos_topic"),
            "goal_pos_raw_frame": LaunchConfiguration("goal_pos_raw_frame"),
            "map_frame": LaunchConfiguration("map_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "fallback_base_frame": LaunchConfiguration("fallback_base_frame"),
            "enable_goal_pos_raw_bridge": LaunchConfiguration("enable_goal_pos_raw_bridge"),
            "publish_target_map": "false",
            "debug_export_point_pairs": LaunchConfiguration("debug_export_point_pairs"),
        }.items(),
    )

    return LaunchDescription(launch_args + [bridge])
