cmake_minimum_required(VERSION 3.16)
project(buff_hitter)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 尋找依賴
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(auto_aim_common REQUIRED)
find_package(gimbal_driver REQUIRED)

# 系統庫依賴
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)  # 建議讓系統自己找，不要 set(Ceres_DIR ...)
find_package(Sophus REQUIRED)
find_package(OpenVINO CONFIG REQUIRED COMPONENTS Runtime)
find_package(Boost REQUIRED COMPONENTS system filesystem coroutine context)
find_package(fmt REQUIRED)

# 2. 自動收集源代碼
# 根據你的 tree 結構，buff_hitter 有 utils 和 module，必須加進來
file(GLOB_RECURSE SRCS 
    "src/*.cpp" 
    "module/*.cpp" 
    "utils/*.cpp"
    "main.cpp"
)

add_executable(${PROJECT_NAME}_node ${SRCS})

# 3. 包含路徑
target_include_directories(${PROJECT_NAME}_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)

# 4. 連結庫
# 非 ROS 的庫用 target_link_libraries
target_link_libraries(${PROJECT_NAME}_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Ceres::ceres
    Sophus::Sophus
    openvino::runtime
    fmt::fmt
    Boost::system Boost::filesystem Boost::coroutine Boost::context
)

# ROS 的庫用 ament_target_dependencies (它會自動處理 include 路徑！)
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    ament_index_cpp
    std_msgs
    sensor_msgs
    cv_bridge
    auto_aim_common  # 這樣就不用自己去 get_filename_component 了
    gimbal_driver
)

# 5. 安裝規則 (Explicit Include)
# 安裝執行檔
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# [重要] 安裝設定檔 (因為你有 config 資料夾)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
    )
endif()

# 6. 導出依賴
ament_export_dependencies(
    rclcpp 
    auto_aim_common 
    OpenCV
    Ceres
    Sophus
)

ament_package()
