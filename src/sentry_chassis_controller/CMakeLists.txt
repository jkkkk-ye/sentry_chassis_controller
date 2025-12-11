cmake_minimum_required(VERSION 3.0.2)
project(sentry_chassis_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  control_toolbox
  hardware_interface
  pluginlib
  dynamic_reconfigure
  tf
  nav_msgs
  geometry_msgs
  urdf
  angles
  realtime_tools
)

generate_dynamic_reconfigure_options(
  cfg/SentryChassis.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    controller_interface
    control_toolbox
    hardware_interface
    pluginlib
    dynamic_reconfigure
    tf
    nav_msgs
    geometry_msgs
    urdf
    angles
    realtime_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sentry_chassis_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS scripts/test_mode_switch.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
