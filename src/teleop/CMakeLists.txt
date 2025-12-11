cmake_minimum_required(VERSION 2.8.3)
project(teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop src/teleop_twist_keyboard.cpp)
target_link_libraries(teleop ${catkin_LIBRARIES})

install(TARGETS teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
