from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_args = [
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
        DeclareLaunchArgument("preferred_distance_cm", default_value="100"),
        DeclareLaunchArgument("distance_deadband_cm", default_value="50"),
        DeclareLaunchArgument("stop_when_no_target", default_value="true"),
        DeclareLaunchArgument("allow_reverse_goal", default_value="false"),
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
                    }
                ],
            )
        ]
    )
