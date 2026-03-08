cmake_minimum_required(VERSION 3.16)
project(behavior_tree)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================================
# 1. 查找依賴 (Find Packages)
# =========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED) # BT v4
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)    # 根據你的要求保留
find_package(ament_index_cpp REQUIRED)  # 用於相對路徑

# 自定義消息包
find_package(auto_aim_common REQUIRED)
find_package(gimbal_driver REQUIRED)

# 系統庫 (與 ROS 1 保持一致)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# =========================================
# 2. 添加子模組 (Logger)
# =========================================
# [關鍵修復] 根據 ROS 1 舊文件，這裡必須編譯 Logger
# 請確保你的專案根目錄下有 Logger 文件夾，且裡面有 CMakeLists.txt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Logger/CMakeLists.txt")
  add_subdirectory(Logger)
endif()

# =========================================
# 3. 編譯可執行檔 (Build Executable)
# =========================================
# 自動抓取 src 下所有 .cpp 以及根目錄的 main.cpp
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS "src/*.cpp")

add_executable(${PROJECT_NAME}_node 
  main.cpp
  ${SRC_FILES}
)

# 設置頭文件路徑
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} # 顯式包含 Boost 頭文件路徑
)

# =========================================
# 4. 連結庫 (Link Libraries)
# =========================================
target_link_libraries(${PROJECT_NAME}_node
  fmt::fmt
  Boost::system
  Boost::filesystem
  behaviortree_cpp::behaviortree_cpp
  
  # [關鍵修復] 連結舊專案的 Logger 庫
  # 假設 Logger/CMakeLists.txt 裡定義的 library 名字叫 Utils::Logger
  Utils::Logger 
)

# 連結 ROS 2 相關依賴
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  auto_aim_common
  gimbal_driver
  ament_index_cpp
)

# =========================================
# 5. 安裝規則 (Install Rules)
# =========================================
# 安裝執行檔
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝 Scripts (XML/Json)，讓 ament_index 能找到
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Scripts")
  install(DIRECTORY Scripts
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 安裝 Launch 檔
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 安裝 Include (如果需要給別人用)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

# =========================================
# 6. 導出依賴 (Export Dependencies)
# =========================================
ament_export_dependencies(
  rclcpp 
  behaviortree_cpp 
  std_msgs 
  geometry_msgs
  auto_aim_common 
  gimbal_driver
  ament_index_cpp
)

ament_package()
