cmake_minimum_required(VERSION 3.0.2)
project(search_mission)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  tf
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES search_mission_node
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs mavros_msgs tf std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(mission_fsm_node src/mission_fsm.cpp)
target_link_libraries(mission_fsm_node ${catkin_LIBRARIES})