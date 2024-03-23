cmake_minimum_required(VERSION 3.0.2)
project(robot_client)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  robots_interface_server
  roscpp
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  robot_client_node
  src/robot_client_node.cpp
  src/robot_client.cpp
  include/robot_client.hpp
)
add_dependencies(robot_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_client_node ${catkin_LIBRARIES})