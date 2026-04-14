import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

try:
    import yaml
except Exception:
    yaml = None


def _bool_default(value) -> str:
    return "true" if bool(value) else "false"


def _load_bridge_defaults(param_file: str) -> dict:
    if yaml is None:
        return {}
    if not param_file or not os.path.exists(param_file):
        return {}
    try:
        with open(param_file, encoding="utf-8") as fh:
            root = yaml.safe_load(fh) or {}
        node_cfg = root.get("target_rel_to_goal_pos_node", {})
        ros_params = node_cfg.get("ros__parameters", {})
        if isinstance(ros_params, dict):
            return ros_params
    except Exception as ex:
        print(f"[target_rel_to_goal_pos] failed to load defaults from '{param_file}': {ex}")
    return {}


def _guess_workspace_root_from_share(share_dir: str) -> str:
    path = Path(share_dir).resolve()
    for parent in [path, *path.parents]:
        if parent.name == "install":
            return str(parent.parent)
    return str(path.parent)


def generate_launch_description():
    bridge_share = get_package_share_directory("navi_tf_bridge")
    default_bridge_param_file = os.path.join(
        bridge_share, "config", "tf_config.yaml"
    )
    workspace_root = _guess_workspace_root_from_share(bridge_share)
    default_area_header_file = os.path.join(
        workspace_root, "src", "behavior_tree", "module", "Area.hpp"
    )
    default_debug_pair_file = os.path.join(
        workspace_root, "src", "navi_tf_bridge", "config", "tf_point_pairs.yaml"
    )
    bridge_defaults = _load_bridge_defaults(default_bridge_param_file)
    get_default = lambda key, fallback: bridge_defaults.get(key, fallback)

    launch_args = [
        DeclareLaunchArgument(
            "input_topic",
            default_value=str(get_default("input_topic", "/ly/navi/target_rel")),
        ),
        DeclareLaunchArgument(
            "output_goal_pos_topic",
            default_value=str(get_default("output_goal_pos_topic", "/ly/navi/goal_pos")),
        ),
        DeclareLaunchArgument(
            "input_goal_pos_raw_topic",
            default_value=str(get_default("input_goal_pos_raw_topic", "/ly/navi/goal_pos_raw")),
        ),
        DeclareLaunchArgument(
            "output_target_map_topic",
            default_value=str(get_default("output_target_map_topic", "/ly/navi/target_map")),
        ),
        DeclareLaunchArgument("map_frame", default_value=str(get_default("map_frame", "map"))),
        DeclareLaunchArgument(
            "base_frame", default_value=str(get_default("base_frame", "base_link"))
        ),
        DeclareLaunchArgument(
            "fallback_base_frame",
            default_value=str(get_default("fallback_base_frame", "baselink")),
        ),
        DeclareLaunchArgument(
            "use_msg_frame_id",
            default_value=_bool_default(get_default("use_msg_frame_id", True)),
        ),
        DeclareLaunchArgument(
            "publish_target_map",
            default_value=_bool_default(get_default("publish_target_map", True)),
        ),
        DeclareLaunchArgument(
            "invert_y_axis",
            default_value=_bool_default(get_default("invert_y_axis", False)),
        ),
        DeclareLaunchArgument(
            "y_axis_max_cm", default_value=str(int(get_default("y_axis_max_cm", 1500)))
        ),
        DeclareLaunchArgument(
            "preferred_distance_cm",
            default_value=str(int(get_default("preferred_distance_cm", 100))),
        ),
        DeclareLaunchArgument(
            "distance_deadband_cm",
            default_value=str(int(get_default("distance_deadband_cm", 50))),
        ),
        DeclareLaunchArgument(
            "stop_when_no_target",
            default_value=_bool_default(get_default("stop_when_no_target", True)),
        ),
        DeclareLaunchArgument(
            "allow_reverse_goal",
            default_value=_bool_default(get_default("allow_reverse_goal", False)),
        ),
        DeclareLaunchArgument(
            "enable_goal_pos_raw_bridge",
            default_value=_bool_default(get_default("enable_goal_pos_raw_bridge", True)),
        ),
        DeclareLaunchArgument(
            "goal_pos_raw_frame",
            default_value=str(get_default("goal_pos_raw_frame", "map")),
        ),
        DeclareLaunchArgument(
            "debug_export_point_pairs",
            default_value=_bool_default(get_default("debug_export_point_pairs", True)),
        ),
        DeclareLaunchArgument(
            "debug_points_reference_frame",
            default_value=str(get_default("debug_points_reference_frame", "map")),
        ),
        DeclareLaunchArgument(
            "debug_area_header_file",
            default_value=str(
                get_default("debug_area_header_file", default_area_header_file)
                or default_area_header_file
            ),
        ),
        DeclareLaunchArgument(
            "debug_point_pairs_output_file",
            default_value=str(
                get_default("debug_point_pairs_output_file", default_debug_pair_file)
                or default_debug_pair_file
            ),
        ),
    ]

    return LaunchDescription(
        launch_args
        + [
            Node(
                package="navi_tf_bridge",
                executable="target_rel_to_goal_pos_node",
                name="target_rel_to_goal_pos_node",
                output="screen",
                parameters=[
                    default_bridge_param_file,
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "input_goal_pos_raw_topic": LaunchConfiguration("input_goal_pos_raw_topic"),
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
                            LaunchConfiguration("preferred_distance_cm"), value_type=int
                        ),
                        "distance_deadband_cm": ParameterValue(
                            LaunchConfiguration("distance_deadband_cm"), value_type=int
                        ),
                        "stop_when_no_target": ParameterValue(
                            LaunchConfiguration("stop_when_no_target"), value_type=bool
                        ),
                        "allow_reverse_goal": ParameterValue(
                            LaunchConfiguration("allow_reverse_goal"), value_type=bool
                        ),
                        "enable_goal_pos_raw_bridge": ParameterValue(
                            LaunchConfiguration("enable_goal_pos_raw_bridge"), value_type=bool
                        ),
                        "goal_pos_raw_frame": LaunchConfiguration("goal_pos_raw_frame"),
                        "debug_export_point_pairs": ParameterValue(
                            LaunchConfiguration("debug_export_point_pairs"), value_type=bool
                        ),
                        "debug_points_reference_frame": LaunchConfiguration(
                            "debug_points_reference_frame"
                        ),
                        "debug_area_header_file": LaunchConfiguration("debug_area_header_file"),
                        "debug_point_pairs_output_file": LaunchConfiguration(
                            "debug_point_pairs_output_file"
                        ),
                    }
                ],
            )
        ]
    )
