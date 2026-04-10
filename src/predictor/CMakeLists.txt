cmake_minimum_required(VERSION 3.16)
project(predictor)

# 1. Basic Setup
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(auto_aim_common REQUIRED)
find_package(gimbal_driver REQUIRED)

# System Dependencies
find_package(fmt 6.1.2 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

# 3. Build Library (Do this FIRST)
# Collect source files
file(GLOB_RECURSE LIB_SOURCES 
    src/predictor.cpp
    src/motion_model.cpp
    src/controller.cpp
    src/solver.cpp
)

# Create the shared library "predictor"
add_library(${PROJECT_NAME} SHARED
    ${LIB_SOURCES}
)

# Include directories for the library
# (注意：target_include_directories 必須使用關鍵字 PUBLIC/PRIVATE，這不會衝突)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/controller>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/predictor>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

# Link libraries for the library target
# [修復] 移除了 PUBLIC 關鍵字，以匹配 ament_target_dependencies 的風格
target_link_libraries(${PROJECT_NAME}
    fmt::fmt
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${Ceres_LIBRARIES}
)

# ROS 2 dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    auto_aim_common
    gimbal_driver
)

# 4. Build Executable (Node) (Do this SECOND)
add_executable(${PROJECT_NAME}_node
    predictor_node.cpp
)

# Link the node to the library we just built
# [修復] 這裡也不使用關鍵字
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

# Include directories for the node (usually inherited from lib, but good to be safe)
target_include_directories(${PROJECT_NAME}_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)

# ROS 2 dependencies for the node
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    std_msgs
    auto_aim_common
    gimbal_driver
)

# 5. Install Rules (Do this LAST)
# Install both the library and the executable
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 6. Export Dependencies (For other packages to use this one)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
    rclcpp 
    std_msgs 
    auto_aim_common 
    gimbal_driver
    Eigen3
    OpenCV
)

ament_package()
