# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

# 装甲板和车辆的匹配器

std_msgs/Header header

CarTracker[] car_trackers
ArmorTracker[] armor_trackers

#carTracker和armorTracers改了聲名

# 当前云台角数据，后续赋值给location
float32 yaw
float32 pitch