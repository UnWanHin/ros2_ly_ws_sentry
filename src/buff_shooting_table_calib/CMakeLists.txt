cmake_minimum_required(VERSION 3.16)
project(buff_shooting_table_calib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gimbal_driver REQUIRED)
find_package(auto_aim_common REQUIRED)

install(PROGRAMS
  script/buff_shooting_table_calib_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME buff_shooting_table_calib_node
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclpy
  std_msgs
  gimbal_driver
  auto_aim_common
)

ament_package()
