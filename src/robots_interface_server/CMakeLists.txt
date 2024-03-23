cmake_minimum_required(VERSION 3.0.2)
project(robots_interface_server)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_service_files(
  FILES
  MoveToPosition.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  position_control_node
  src/position_control_node.cpp
  src/robot_interface.cpp
  include/robot_interface.hpp
)
target_link_libraries(position_control_node ${catkin_LIBRARIES})

