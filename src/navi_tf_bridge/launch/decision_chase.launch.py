#!/usr/bin/env python3

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_bt_config_path(behavior_tree_share: str, configured_path: str) -> str:
    if not configured_path:
        return ""
    path = Path(configured_path)
    if path.is_absolute():
        return str(path)
    return str((Path(behavior_tree_share) / path).resolve(strict=False))


def _normalize_bool(raw: str) -> str:
    value = (raw or "").strip().lower()
    if value in ("1", "true", "yes", "on"):
        return "true"
    if value in ("0", "false", "no", "off"):
        return "false"
    return ""


def _resolve_bridge_chase_params(context, behavior_tree_share: str):
    resolved_preferred_distance_cm = "100"
    resolved_distance_deadband_cm = "50"
    resolved_stop_when_no_target = "true"

    bt_config_file_value = LaunchConfiguration("bt_config_file").perform(context).strip()
    bt_config_path = _resolve_bt_config_path(behavior_tree_share, bt_config_file_value)
    if bt_config_path and os.path.exists(bt_config_path):
        try:
            with open(bt_config_path, encoding="utf-8") as fh:
                config_root = json.load(fh)
            chase_cfg = config_root.get("Chase", {})
            if isinstance(chase_cfg, dict):
                resolved_preferred_distance_cm = str(
                    int(chase_cfg.get("PreferredDistanceCm", int(resolved_preferred_distance_cm)))
                )
                resolved_distance_deadband_cm = str(
                    int(chase_cfg.get("DistanceDeadbandCm", int(resolved_distance_deadband_cm)))
                )
                resolved_stop_when_no_target = (
                    "true" if bool(chase_cfg.get("StopWhenNoTarget", True)) else "false"
                )
        except Exception as ex:
            print(
                f"[decision_chase] failed to parse bt_config_file '{bt_config_path}': {ex}. "
                "Bridge chase parameters fall back to defaults."
            )

    preferred_override = LaunchConfiguration("preferred_distance_cm").perform(context).strip()
    if preferred_override:
        resolved_preferred_distance_cm = preferred_override

    deadband_override = LaunchConfiguration("distance_deadband_cm").perform(context).strip()
    if deadband_override:
        resolved_distance_deadband_cm = deadband_override

    stop_override = _normalize_bool(LaunchConfiguration("stop_when_no_target").perform(context))
    if stop_override:
        resolved_stop_when_no_target = stop_override

    allow_reverse_goal = _normalize_bool(LaunchConfiguration("allow_reverse_goal").perform(context))
    if not allow_reverse_goal:
        allow_reverse_goal = "false"

    return [
        SetLaunchConfiguration(
            "resolved_bridge_preferred_distance_cm", resolved_preferred_distance_cm
        ),
        SetLaunchConfiguration(
            "resolved_bridge_distance_deadband_cm", resolved_distance_deadband_cm
        ),
        SetLaunchConfiguration(
            "resolved_bridge_stop_when_no_target", resolved_stop_when_no_target
        ),
        SetLaunchConfiguration("resolved_bridge_allow_reverse_goal", allow_reverse_goal),
    ]


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
        DeclareLaunchArgument(
            "preferred_distance_cm",
            default_value="",
            description="Optional bridge override. Empty means load Chase.PreferredDistanceCm from bt_config_file.",
        ),
        DeclareLaunchArgument(
            "distance_deadband_cm",
            default_value="",
            description="Optional bridge override. Empty means load Chase.DistanceDeadbandCm from bt_config_file.",
        ),
        DeclareLaunchArgument(
            "stop_when_no_target",
            default_value="",
            description="Optional bridge override. Empty means load Chase.StopWhenNoTarget from bt_config_file.",
        ),
        DeclareLaunchArgument(
            "allow_reverse_goal",
            default_value="false",
            description="Whether the bridge may output a reverse chase goal when already too close to the target.",
        ),
        DeclareLaunchArgument("resolved_bridge_preferred_distance_cm", default_value="100"),
        DeclareLaunchArgument("resolved_bridge_distance_deadband_cm", default_value="50"),
        DeclareLaunchArgument("resolved_bridge_stop_when_no_target", default_value="true"),
        DeclareLaunchArgument("resolved_bridge_allow_reverse_goal", default_value="false"),
        OpaqueFunction(function=_resolve_bridge_chase_params, args=[behavior_tree_share]),
        LogInfo(
            msg=[
                "[decision_chase] bridge preferred_distance_cm: ",
                LaunchConfiguration("resolved_bridge_preferred_distance_cm"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge distance_deadband_cm: ",
                LaunchConfiguration("resolved_bridge_distance_deadband_cm"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge stop_when_no_target: ",
                LaunchConfiguration("resolved_bridge_stop_when_no_target"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge allow_reverse_goal: ",
                LaunchConfiguration("resolved_bridge_allow_reverse_goal"),
            ]
        ),
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
                "use_msg_frame_id": ParameterValue(
                    LaunchConfiguration("use_msg_frame_id"), value_type=bool
                ),
                "publish_target_map": ParameterValue(
                    LaunchConfiguration("publish_target_map"), value_type=bool
                ),
                "invert_y_axis": ParameterValue(
                    LaunchConfiguration("invert_y_axis"), value_type=bool
                ),
                "y_axis_max_cm": ParameterValue(
                    LaunchConfiguration("y_axis_max_cm"), value_type=int
                ),
                "preferred_distance_cm": ParameterValue(
                    LaunchConfiguration("resolved_bridge_preferred_distance_cm"), value_type=int
                ),
                "distance_deadband_cm": ParameterValue(
                    LaunchConfiguration("resolved_bridge_distance_deadband_cm"), value_type=int
                ),
                "stop_when_no_target": ParameterValue(
                    LaunchConfiguration("resolved_bridge_stop_when_no_target"), value_type=bool
                ),
                "allow_reverse_goal": ParameterValue(
                    LaunchConfiguration("resolved_bridge_allow_reverse_goal"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(launch_args + [chase_only, target_rel_bridge])
