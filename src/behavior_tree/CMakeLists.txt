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
get_filename_component(BTCPP_WS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(BTCPP_WS_ROOT_DIR "${BTCPP_WS_SRC_DIR}" DIRECTORY)
set(BTCPP_V4_ROOT "${BTCPP_WS_ROOT_DIR}/third_party/behaviortree_cpp_v4/install"
  CACHE PATH "Install prefix of pinned BehaviorTree.CPP v4 (defaults to <ws>/third_party/behaviortree_cpp_v4/install)")
set(BTCPP_V4_CMAKE_DIR_SHARE "${BTCPP_V4_ROOT}/share/behaviortree_cpp/cmake")
set(BTCPP_V4_CMAKE_DIR_LIB "${BTCPP_V4_ROOT}/lib/cmake/behaviortree_cpp")
set(BTCPP_V4_CMAKE_PATHS "")
set(BTCPP_IS_PINNED FALSE)
set(BTCPP_SYSTEM_PREFIX_PATHS "")

if(EXISTS "${BTCPP_V4_CMAKE_DIR_SHARE}/behaviortree_cppConfig.cmake")
  list(APPEND BTCPP_V4_CMAKE_PATHS "${BTCPP_V4_CMAKE_DIR_SHARE}")
endif()
if(EXISTS "${BTCPP_V4_CMAKE_DIR_LIB}/behaviortree_cppConfig.cmake")
  list(APPEND BTCPP_V4_CMAKE_PATHS "${BTCPP_V4_CMAKE_DIR_LIB}")
endif()

if(BTCPP_V4_CMAKE_PATHS STREQUAL "")
  if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    string(REPLACE ":" ";" BTCPP_AMENT_PREFIX_PATHS "$ENV{AMENT_PREFIX_PATH}")
    list(APPEND BTCPP_SYSTEM_PREFIX_PATHS ${BTCPP_AMENT_PREFIX_PATHS})
  endif()
  if(DEFINED ENV{CMAKE_PREFIX_PATH} AND NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
    string(REPLACE ":" ";" BTCPP_CMAKE_PREFIX_PATHS "$ENV{CMAKE_PREFIX_PATH}")
    list(APPEND BTCPP_SYSTEM_PREFIX_PATHS ${BTCPP_CMAKE_PREFIX_PATHS})
  endif()
  if(DEFINED ENV{ROS_DISTRO} AND EXISTS "/opt/ros/$ENV{ROS_DISTRO}")
    list(APPEND BTCPP_SYSTEM_PREFIX_PATHS "/opt/ros/$ENV{ROS_DISTRO}")
  endif()
  list(REMOVE_DUPLICATES BTCPP_SYSTEM_PREFIX_PATHS)

  if(BTCPP_SYSTEM_PREFIX_PATHS)
    find_package(behaviortree_cpp CONFIG QUIET
      PATHS ${BTCPP_SYSTEM_PREFIX_PATHS}
    )
  else()
    find_package(behaviortree_cpp CONFIG QUIET)
  endif()

  if(NOT behaviortree_cpp_FOUND)
    set(BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY "(default CMake search paths only)")
    if(BTCPP_SYSTEM_PREFIX_PATHS)
      string(REPLACE ";" "\n  - " BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY "${BTCPP_SYSTEM_PREFIX_PATHS}")
      set(BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY "  - ${BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY}")
    endif()
    message(FATAL_ERROR
      "BehaviorTree.CPP v4+ not found.\n"
      "Checked pinned workspace paths:\n"
      "  - ${BTCPP_V4_CMAKE_DIR_SHARE}\n"
      "  - ${BTCPP_V4_CMAKE_DIR_LIB}\n"
      "Checked system/package-manager prefixes:\n"
      "${BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY}\n"
      "Install/copy BehaviorTree.CPP v4 into ${BTCPP_V4_ROOT}, or install the system package.")
  endif()

  message(STATUS "BehaviorTree.CPP source: system package fallback")
  if(DEFINED behaviortree_cpp_DIR)
    message(STATUS "BehaviorTree.CPP config dir: ${behaviortree_cpp_DIR}")
  endif()
else()
  find_package(behaviortree_cpp CONFIG REQUIRED
    PATHS ${BTCPP_V4_CMAKE_PATHS}
    NO_DEFAULT_PATH
  )
  set(BTCPP_IS_PINNED TRUE)
  message(STATUS "BehaviorTree.CPP source: pinned workspace copy")
  message(STATUS "Pinned BehaviorTree.CPP root: ${BTCPP_V4_ROOT}")
endif()

if(NOT DEFINED behaviortree_cpp_VERSION OR behaviortree_cpp_VERSION VERSION_LESS 4.0.0)
  message(FATAL_ERROR
    "behavior_tree requires BehaviorTree.CPP v4+, but found version '${behaviortree_cpp_VERSION}'.")
endif()

message(STATUS "BehaviorTree.CPP version: ${behaviortree_cpp_VERSION}")
if(TARGET behaviortree_cpp::behaviortree_cpp)
  set(BTCPP_LINK_TARGET behaviortree_cpp::behaviortree_cpp)
elseif(TARGET BT::behaviortree_cpp)
  set(BTCPP_LINK_TARGET BT::behaviortree_cpp)
else()
  message(FATAL_ERROR "No known BT target found. Expected behaviortree_cpp::behaviortree_cpp or BT::behaviortree_cpp.")
endif()
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

if(BTCPP_IS_PINNED AND EXISTS "${BTCPP_V4_ROOT}/lib")
  set_target_properties(${PROJECT_NAME}_node PROPERTIES
    BUILD_RPATH "${BTCPP_V4_ROOT}/lib"
    INSTALL_RPATH "${BTCPP_V4_ROOT}/lib"
  )
endif()

# 設置頭文件路徑
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  ${Boost_INCLUDE_DIRS} # 顯式包含 Boost 頭文件路徑
)

# =========================================
# 4. 連結庫 (Link Libraries)
# =========================================
target_link_libraries(${PROJECT_NAME}_node
  fmt::fmt
  Boost::system
  Boost::filesystem
  ${BTCPP_LINK_TARGET}
  
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
