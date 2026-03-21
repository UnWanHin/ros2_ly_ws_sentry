# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

bool tracking  # 是否开始跟踪 ， 替代找到目标IsfindTarget
geometry_msgs/Point position  # xc yc zc
float32 yaw
geometry_msgs/Vector3 velocity # 速度 vx vy vz
float32 v_yaw
float32 radius_1  # 当前预测的半径 r
float32 radius_2  # 上一次预测的半径
float32 z_2 # 上一次预测的z