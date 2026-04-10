#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    behavior_tree_share = get_package_share_directory("behavior_tree")
    buff_share = get_package_share_directory("buff_hitter")
    calib_share = get_package_share_directory("buff_shooting_table_calib")
    config_root = os.path.join(behavior_tree_share, "config")

    default_base_config_file = os.path.join(config_root, "base_config.yaml")
    default_override_config_file = os.path.join(config_root, "override_config.yaml")
    default_buff_config_file = os.path.join(buff_share, "config", "buff_config.yaml")
    default_calib_config_file = os.path.join(
        calib_share, "config", "buff_shooting_table_calib_config.yaml"
    )

    base_config_file = LaunchConfiguration("base_config_file")
    buff_config_file = LaunchConfiguration("buff_config_file")
    config_file = LaunchConfiguration("config_file")
    calib_config_file = LaunchConfiguration("calib_config_file")
    calib_mode = LaunchConfiguration("calib_mode")
    record_dir = LaunchConfiguration("record_dir")
    csv_strategy = LaunchConfiguration("csv_strategy")
    csv_path = LaunchConfiguration("csv_path")
    require_valid_debug = LaunchConfiguration("require_valid_debug")
    sample_on_rising_edge = LaunchConfiguration("sample_on_rising_edge")
    min_sample_interval_sec = LaunchConfiguration("min_sample_interval_sec")
    output = LaunchConfiguration("output")

    return LaunchDescription(
        [
            DeclareLaunchArgument("base_config_file", default_value=default_base_config_file),
            DeclareLaunchArgument("buff_config_file", default_value=default_buff_config_file),
            DeclareLaunchArgument("config_file", default_value=default_override_config_file),
            DeclareLaunchArgument("calib_config_file", default_value=default_calib_config_file),
            DeclareLaunchArgument("calib_mode", default_value="static"),
            DeclareLaunchArgument("record_dir", default_value="~/workspace/record"),
            DeclareLaunchArgument("csv_strategy", default_value="new"),
            DeclareLaunchArgument("csv_path", default_value=""),
            DeclareLaunchArgument("require_valid_debug", default_value="true"),
            DeclareLaunchArgument("sample_on_rising_edge", default_value="true"),
            DeclareLaunchArgument("min_sample_interval_sec", default_value="0.08"),
            DeclareLaunchArgument("output", default_value="screen"),
            LogInfo(msg=["[buff_calib] base_config: ", base_config_file]),
            LogInfo(msg=["[buff_calib] buff_config: ", buff_config_file]),
            LogInfo(msg=["[buff_calib] override_config: ", config_file]),
            LogInfo(msg=["[buff_calib] calib_config: ", calib_config_file]),
            LogInfo(msg=["[buff_calib] calib_mode: ", calib_mode]),
            LogInfo(msg=["[buff_calib] record_dir: ", record_dir]),
            LogInfo(msg=["[buff_calib] csv_strategy: ", csv_strategy]),
            LogInfo(msg=["[buff_calib] csv_path: ", csv_path]),
            Node(
                package="buff_shooting_table_calib",
                executable="buff_shooting_table_calib_node",
                name="buff_shooting_table_calib",
                output=output,
                parameters=[
                    base_config_file,
                    buff_config_file,
                    calib_config_file,
                    config_file,
                    {
                        "buff_shooting_table_calib.calib_mode": calib_mode,
                        "buff_shooting_table_calib.record_dir": record_dir,
                        "buff_shooting_table_calib.csv_strategy": csv_strategy,
                        "buff_shooting_table_calib.csv_path": csv_path,
                        "buff_shooting_table_calib.require_valid_debug": require_valid_debug,
                        "buff_shooting_table_calib.sample_on_rising_edge": sample_on_rising_edge,
                        "buff_shooting_table_calib.min_sample_interval_sec": min_sample_interval_sec,
                    },
                ],
            ),
        ]
    )
