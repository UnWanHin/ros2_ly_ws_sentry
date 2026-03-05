#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('detector'),
        'config',
        'auto_aim_config.yaml'
    )
    
    print(f"\033[92m[前哨模式] 載入配置: {config_file}\033[0m")
    
    return LaunchDescription([
        Node(
            package='gimbal_driver',
            executable='gimbal_driver_node',
            name='gimbal_driver',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='outpost_hitter',
            executable='outpost_hitter_node',
            name='outpost_hitter',
            output='screen',
            parameters=[config_file]
        ),
    ])
