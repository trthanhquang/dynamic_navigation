cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  roscpp
  rospy
  tf
  message_generation
)

add_message_files(
  FILES
  PoseVel.msg
  PoseVelArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_planner_node src/path_planner.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES})

add_executable(adstar_planner_node src/adstar_planner.cpp)
target_link_libraries(adstar_planner_node ${catkin_LIBRARIES})
