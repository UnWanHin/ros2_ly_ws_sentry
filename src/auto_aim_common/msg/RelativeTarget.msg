# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

std_msgs/Header header

# Whether this relative target is currently valid.
bool valid

# Relative target point in robot/chassis frame (meters).
# Convention: x-forward, y-left, z-up.
float32 x
float32 y
float32 z

# Extra chase context for downstream controller.
float32 distance_m
float32 yaw_error_deg
float32 pitch_error_deg
uint8 armor_type
uint8 aim_mode
