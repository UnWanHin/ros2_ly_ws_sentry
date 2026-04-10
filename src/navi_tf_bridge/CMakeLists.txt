cmake_minimum_required(VERSION 3.16)
project(navi_tf_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(auto_aim_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(target_rel_to_goal_pos_node
  src/target_rel_to_goal_pos_node.cpp
)

ament_target_dependencies(target_rel_to_goal_pos_node
  rclcpp
  std_msgs
  geometry_msgs
  auto_aim_common
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS target_rel_to_goal_pos_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
