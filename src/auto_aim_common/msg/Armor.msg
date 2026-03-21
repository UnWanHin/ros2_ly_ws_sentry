# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

# Armor.msg
# uint8 BLUE = 0
# uint8 RED = 1
# uint8 GRAY = 2
# uint8 OTHER = 3

int32 type  # 装甲板数字类型
int32 color  # 装甲板颜色
float32 distance 
float32 distance_to_image_center# 装甲板的识别中心到图像中心的距离
float32[] corners_x # 装甲板的四个点在图像坐标系下的x坐标
float32[] corners_y # 装甲板的四个点在图像坐标系下的y坐标

# geometry_msgs/Pose pose  # 装甲板再相机坐标系下面的xyz以及用四元数表示的方向
# pnp解算的变换向量
float32[] rotation
float32[] translation



# float32 prob
# geometry_msgs/Point32[] points
# int32 area
# geometry_msgs/Point32[4] apex
