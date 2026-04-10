cmake_minimum_required(VERSION 3.16)
project(auto_aim_common)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找依賴
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fmt REQUIRED) 

# 2. 生成消息接口 (這部分保持不變)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  "msg/RelativeTarget.msg"
  "msg/DebugFilter.msg"
  "msg/Rect.msg"
  "msg/Car.msg"
  "msg/ArmorTracker.msg"
  "msg/CarTracker.msg"
  "msg/Trackers.msg"
  "msg/AngleImage.msg"
  "msg/BuffDebug.msg"
  "msg/PredictorArmorVis.msg"
  "msg/PredictorCarVis.msg"
  "msg/PredictorVis.msg"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

# 3. [關鍵] 保留 INTERFACE 庫定義
# 這樣做可以確保你原本代碼中對 auto_aim_common 的依賴邏輯完全不變
add_library(${PROJECT_NAME}_interface INTERFACE)

# 設置 Interface 的 Include 目錄
target_include_directories(${PROJECT_NAME}_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 設置 Interface 的連結庫 (傳遞 fmt 依賴)
target_link_libraries(${PROJECT_NAME}_interface INTERFACE
  fmt::fmt
)

# 設置 Interface 的編譯定義
target_compile_definitions(${PROJECT_NAME}_interface INTERFACE
  FMT_HEADER_ONLY
)

# 4. 安裝 Interface 目標 (導出給別的包用)
install(TARGETS ${PROJECT_NAME}_interface
  EXPORT ${PROJECT_NAME}_targets  # 導出名為 auto_aim_common_targets
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝頭文件
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING 
    PATTERN "*.hpp"
    PATTERN "*.h"
)

# 5. 導出依賴 (ament_package 之前)
# 導出我們剛定義的 target，這樣下游包 find_package 後就能用
ament_export_targets(${PROJECT_NAME}_targets)

# 導出其他依賴，確保下游包知道要連結這些
ament_export_dependencies(geometry_msgs rclcpp rclpy std_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
