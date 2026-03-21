# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

# Armors.msg
# 时间戳
std_msgs/Header header

# 装甲板信息列表
Armor[] armors
Car[] cars # 车信息列表

# 将当前图像解算的数据和实际的串口数据进行绑定解决延时的问题
float32 yaw
float32 pitch

# 给预测器的可用装甲板
bool is_available_armor_for_predictor
int16 target_armor_index_for_predictor
