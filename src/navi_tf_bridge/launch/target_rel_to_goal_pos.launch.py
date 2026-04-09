from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="navi_tf_bridge",
                executable="target_rel_to_goal_pos_node",
                name="target_rel_to_goal_pos_node",
                output="screen",
            )
        ]
    )
