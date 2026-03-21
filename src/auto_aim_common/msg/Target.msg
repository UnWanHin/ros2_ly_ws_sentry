# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

# 时间戳
std_msgs/Header header
# 跟踪器返回的目标消息
# string id  # 返回的跟踪目标，暂时弃用
bool status  # 表示跟踪状态，在打符的时候用于控制是否发弹
bool buff_follow # 打符进行角度跟踪
float32 yaw
float32 pitch  # 俯仰角