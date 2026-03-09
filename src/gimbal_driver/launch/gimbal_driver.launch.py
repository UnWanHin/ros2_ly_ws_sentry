#!/usr/bin/env python3
"""
gimbal_driver 独立启动入口。

用途：
- 单独调试串口收发和 /ly/control/* -> /ly/gimbal/* 转发行为。

关键参数：
- use_virtual_device：是否使用虚拟设备（离车调试建议 true）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output = LaunchConfiguration("output")
    use_virtual_device = LaunchConfiguration("use_virtual_device")

    return LaunchDescription([
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
        Node(
            package="gimbal_driver",
            executable="gimbal_driver_node",
            name="gimbal_driver",
            output=output,
            parameters=[{
                "io_config/use_virtual_device": use_virtual_device
            }],
        ),
    ])
