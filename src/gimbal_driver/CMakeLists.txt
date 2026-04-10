cmake_minimum_required(VERSION 3.16)  #ros2??C?n3.16
project(gimbal_driver)

# if(POLICY CMP0057)
#   cmake_policy(SET CMP0057 NEW)
# endif()
# if(POLICY CMP0074)
#   cmake_policy(SET CMP0074 NEW)
# endif()


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#         roscpp
#         std_msgs
#         message_generation
# )
#ament_cmake

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) #ros2?s?W
find_package(rclcpp REQUIRED) #ros2?s?W
find_package(std_msgs REQUIRED) #ros2?s?W
find_package(Boost CONFIG REQUIRED COMPONENTS system)
#find_package(Boost REQUIRED COMPONENTS system)
find_package(fmt 6.1.2 CONFIG REQUIRED)
find_package(geometry_msgs REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder

# add_message_files(
#         FILES
#         GimbalAngles.msg
#         UWBPos.msg
#         Vel.msg
#         Health.msg
#         GameData.msg
#         BuffData.msg
#         PositionData.msg
# )

# generate_messages(
#         DEPENDENCIES
#         std_msgs
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalAngles.msg"
  "msg/GimbalYaw.msg"
  "msg/UWBPos.msg"
  "msg/Vel.msg"
  "msg/Health.msg"
  "msg/GameData.msg"
  "msg/BuffData.msg"
  "msg/PositionData.msg"
  DEPENDENCIES geometry_msgs std_msgs
)


#ros2 ?N??message_generation?Mmessage_runtime  

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# include_directories(
#         # include
#         ${catkin_INCLUDE_DIRS}
# )



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/gimbal_driver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/gimbal_driver_node.cpp)
add_executable(${PROJECT_NAME}_node main.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c>
  ${ament_INCLUDE_DIRS}
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  #${rclcpp_LIBRARIES}
  Boost::system
  fmt::fmt
  #${PROJECT_NAME}
  "${cpp_typesupport_target}"
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# target_link_libraries(${PROJECT_NAME}_node
  
# )

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

# install(TARGETS ${PROJECT_NAME}_node
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY module
  DESTINATION include/${PROJECT_NAME}
)
#ROS2 ?N??catkin_install_python?Minstall(TARGETS ...)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gimbal_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# catkin_package(
#         #  INCLUDE_DIRS include
#         #  LIBRARIES gimbal_driver
#         CATKIN_DEPENDS roscpp std_msgs message_runtime
#         #  DEPENDS system_lib
# )
# ament_export_dependencies(rosidl_default_runtime)
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME}_node)
ament_export_dependencies(rosidl_default_runtime)

#ament_export_libraries(${PROJECT_NAME}_node)
ament_export_dependencies(rclcpp std_msgs geometry_msgs)


ament_package() #ros2 ?N??catkin_package????K?[?b????
