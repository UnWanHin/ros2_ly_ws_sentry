# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

std_msgs/Header header

# Whether current buff solution is valid.
bool status

# Buff mode reported by solver: 1=small, 2=big.
uint8 mode

# Solver baseline setpoint before external calibration offset.
float32 target_yaw
float32 target_pitch

# Periodic-model feature (for big buff moving compensation).
float32 rotation_angle

# Geometric features in meters.
float32 distance_m
float32 height_m
float32 target_x_m
float32 target_y_m
float32 target_z_m
