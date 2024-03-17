cmake_minimum_required(VERSION 3.0.2)
project(robots_interface_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_service_files(
  FILES
  GetPosition.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(position_control_node src/position_control_node.cpp)
target_link_libraries(position_control_node ${catkin_LIBRARIES})

