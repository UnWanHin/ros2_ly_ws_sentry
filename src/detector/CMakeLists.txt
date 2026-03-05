cmake_minimum_required(VERSION 3.16)
project(detector)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找依賴
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(auto_aim_common REQUIRED)
find_package(gimbal_driver REQUIRED)

# 系統依賴
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED) # 建議去掉 6.1.2 版本號，Ubuntu 24.04 通常是 fmt 9+
find_package(Sophus CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(OpenVINO CONFIG REQUIRED COMPONENTS Runtime)

add_compile_definitions(FMT_HEADER_ONLY)

# GalaxySDK 設置
add_library(GxIAPI SHARED IMPORTED GLOBAL)
set_property(TARGET GxIAPI PROPERTY IMPORTED_SONAME libgxiapi.so)
set_property(TARGET GxIAPI PROPERTY IMPORTED_LOCATION "/usr/lib/libgxiapi.so")
# [注意] 請確認你的目錄裡真的有 Modules/Linux_x64/inc
# 如果你的相機頭文件其實是在 module 資料夾裡，請修改這裡
set(GalaxySDK_Headers ${CMAKE_CURRENT_SOURCE_DIR}/Modules/Linux_x64/inc)
set(GalaxySDK_LIBRARIES GxIAPI)

# =========================================================
# 2. 編譯庫 (關鍵修復：加入 module 資料夾)
# =========================================================
file(GLOB_RECURSE LIB_SOURCES 
    "src/*.cpp"
    "module/*.cpp"  # [修復] 你的 tree 裡有 module，必須加進來編譯
)

add_library(${PROJECT_NAME} SHARED
    ${LIB_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module> # [修復] 讓代碼能直接 include module 裡的頭文件
    $<INSTALL_INTERFACE:include>
    ${OpenVINO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GalaxySDK_Headers}
    ${Sophus_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    ${GalaxySDK_LIBRARIES}
    ${OpenCV_LIBS}
    openvino::runtime
    Eigen3::Eigen
    Ceres::ceres
    Sophus::Sophus
    fmt::fmt
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    auto_aim_common
    gimbal_driver
)

# 3. 編譯節點 (detector_node.cpp 在根目錄)
add_executable(${PROJECT_NAME}_node
    detector_node.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GalaxySDK_Headers}
    ${OpenCV_INCLUDE_DIRS}
)

# 鏈接上面編譯好的庫
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    auto_aim_common
    gimbal_driver
)

# =========================================================
# 4. 安裝規則 (Install) - 這裡是重點
# =========================================================
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/mapper_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mapper_node
)

install(PROGRAMS
  script/fire_flip_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fire_flip_test
)

install(DIRECTORY include/
    DESTINATION include
)

# [正確] 安裝 launch
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# [正確] 安裝 config
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# [新增] 安裝 Extras (關鍵！這裡存放模型文件)
# 如果不安裝，節點啟動時會找不到 .xml 模型而崩潰
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Extras")
    install(DIRECTORY Extras
        DESTINATION share/${PROJECT_NAME}
    )
endif()

# 5. 導出依賴
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
    rclcpp 
    std_msgs 
    sensor_msgs 
    auto_aim_common 
    cv_bridge
    OpenCV  
    Eigen3
)

ament_package()
