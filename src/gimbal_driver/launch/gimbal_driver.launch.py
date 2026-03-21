#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
gimbal_driver 独立启动入口。

用途：
- 单独调试串口收发和 /ly/control/* -> /ly/gimbal/* 转发行为。

关键参数：
- config_file：共享参数 YAML（可提供串口设备名、波特率等）。
- use_virtual_device：是否使用虚拟设备（离车调试建议 true）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def build_node(context):
        config_file_value = LaunchConfiguration("config_file").perform(context).strip()
        output_value = LaunchConfiguration("output")
        use_virtual_device_value = LaunchConfiguration("use_virtual_device")

        parameters = [{
            "io_config/use_virtual_device": use_virtual_device_value
        }]
        if config_file_value:
            parameters.insert(0, config_file_value)

        return [
            Node(
                package="gimbal_driver",
                executable="gimbal_driver_node",
                name="gimbal_driver",
                output=output_value,
                parameters=parameters,
            ),
        ]

    output = LaunchConfiguration("output")
    use_virtual_device = LaunchConfiguration("use_virtual_device")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Optional YAML config file for gimbal_driver. Empty keeps built-in defaults.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument(
            "use_virtual_device",
            default_value="false",
            description="Whether to use virtual serial device in gimbal_driver.",
        ),
        OpaqueFunction(function=build_node),
    ])
