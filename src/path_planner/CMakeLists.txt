cmake_minimum_required(VERSION 2.8.3)
project(path_planner)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_planner_node src/path_planner.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES})