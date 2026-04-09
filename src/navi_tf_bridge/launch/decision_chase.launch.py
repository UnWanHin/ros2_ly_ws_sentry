#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    behavior_tree_share = get_package_share_directory("behavior_tree")
    detector_share = get_package_share_directory("detector")
    predictor_share = get_package_share_directory("predictor")
    outpost_share = get_package_share_directory("outpost_hitter")
    buff_share = get_package_share_directory("buff_hitter")

    chase_only_launch = os.path.join(behavior_tree_share, "launch", "chase_only.launch.py")
    config_root = os.path.join(behavior_tree_share, "config")

    default_base_config_file = os.path.join(config_root, "base_config.yaml")
    default_override_config_file = os.path.join(config_root, "override_config.yaml")
    default_detector_config_file = os.path.join(detector_share, "config", "detector_config.yaml")
    default_predictor_config_file = os.path.join(predictor_share, "config", "predictor_config.yaml")
    default_outpost_config_file = os.path.join(outpost_share, "config", "outpost_config.yaml")
    default_buff_config_file = os.path.join(buff_share, "config", "buff_config.yaml")

    launch_args = [
        DeclareLaunchArgument("mode", default_value="league"),
        DeclareLaunchArgument("config_file", default_value=default_override_config_file),
        DeclareLaunchArgument("base_config_file", default_value=default_base_config_file),
        DeclareLaunchArgument("detector_config_file", default_value=default_detector_config_file),
        DeclareLaunchArgument("predictor_config_file", default_value=default_predictor_config_file),
        DeclareLaunchArgument("outpost_config_file", default_value=default_outpost_config_file),
        DeclareLaunchArgument("buff_config_file", default_value=default_buff_config_file),
        DeclareLaunchArgument("output", default_value="screen"),
        DeclareLaunchArgument("offline", default_value="false"),
        DeclareLaunchArgument("debug_bypass_is_start", default_value="true"),
        DeclareLaunchArgument("wait_for_game_start_timeout_sec", default_value="0"),
        DeclareLaunchArgument("league_referee_stale_timeout_ms", default_value="0"),
        DeclareLaunchArgument("bt_tree_file", default_value=""),
        DeclareLaunchArgument(
            "bt_config_file", default_value="Scripts/ConfigJson/chase_only_competition.json"
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_detector", default_value="true"),
        DeclareLaunchArgument("use_tracker", default_value="true"),
        DeclareLaunchArgument("use_predictor", default_value="true"),
        DeclareLaunchArgument("use_outpost", default_value="false"),
        DeclareLaunchArgument("use_buff", default_value="false"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument("input_topic", default_value="/ly/navi/target_rel"),
        DeclareLaunchArgument("output_goal_pos_topic", default_value="/ly/navi/goal_pos"),
        DeclareLaunchArgument("output_target_map_topic", default_value="/ly/navi/target_map"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("fallback_base_frame", default_value="baselink"),
        DeclareLaunchArgument("use_msg_frame_id", default_value="true"),
        DeclareLaunchArgument("publish_target_map", default_value="true"),
        DeclareLaunchArgument("invert_y_axis", default_value="false"),
        DeclareLaunchArgument("y_axis_max_cm", default_value="1500"),
    ]

    chase_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(chase_only_launch),
        launch_arguments={
            "mode": LaunchConfiguration("mode"),
            "config_file": LaunchConfiguration("config_file"),
            "base_config_file": LaunchConfiguration("base_config_file"),
            "detector_config_file": LaunchConfiguration("detector_config_file"),
            "predictor_config_file": LaunchConfiguration("predictor_config_file"),
            "outpost_config_file": LaunchConfiguration("outpost_config_file"),
            "buff_config_file": LaunchConfiguration("buff_config_file"),
            "output": LaunchConfiguration("output"),
            "offline": LaunchConfiguration("offline"),
            "debug_bypass_is_start": LaunchConfiguration("debug_bypass_is_start"),
            "wait_for_game_start_timeout_sec": LaunchConfiguration("wait_for_game_start_timeout_sec"),
            "league_referee_stale_timeout_ms": LaunchConfiguration("league_referee_stale_timeout_ms"),
            "bt_tree_file": LaunchConfiguration("bt_tree_file"),
            "bt_config_file": LaunchConfiguration("bt_config_file"),
            "use_gimbal": LaunchConfiguration("use_gimbal"),
            "use_detector": LaunchConfiguration("use_detector"),
            "use_tracker": LaunchConfiguration("use_tracker"),
            "use_predictor": LaunchConfiguration("use_predictor"),
            "use_outpost": LaunchConfiguration("use_outpost"),
            "use_buff": LaunchConfiguration("use_buff"),
            "use_behavior_tree": LaunchConfiguration("use_behavior_tree"),
        }.items(),
    )

    target_rel_bridge = Node(
        package="navi_tf_bridge",
        executable="target_rel_to_goal_pos_node",
        name="target_rel_to_goal_pos_node",
        output=LaunchConfiguration("output"),
        parameters=[
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "output_goal_pos_topic": LaunchConfiguration("output_goal_pos_topic"),
                "output_target_map_topic": LaunchConfiguration("output_target_map_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "fallback_base_frame": LaunchConfiguration("fallback_base_frame"),
                "use_msg_frame_id": LaunchConfiguration("use_msg_frame_id"),
                "publish_target_map": LaunchConfiguration("publish_target_map"),
                "invert_y_axis": LaunchConfiguration("invert_y_axis"),
                "y_axis_max_cm": LaunchConfiguration("y_axis_max_cm"),
            }
        ],
    )

    return LaunchDescription(launch_args + [chase_only, target_rel_bridge])
