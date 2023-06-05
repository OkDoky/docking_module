cmake_minimum_required(VERSION 3.0.2)
project(docking_tracker)

add_compile_options(-std=c++11)
option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  rospy
  tf
  tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  docking_module/include/docking_tracker/nmpc_ros
  docking_module/include/docking_tracker/nmpc_deceleration
  docking_module/include/docking_tracker
)
include_directories(
  /usr/include
  /usr/include/eigen3)


link_directories(/usr/lib)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
    docking_tracker
    docking_tracker/nmpc_ros
    docking_tracker/nmpc_deceleration
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    tf
    tf2_ros
    nav_msgs
)

## init nmpc node
add_library(nmpc_lib
  src/nmpc_ros/mpc_planner_ros.cpp
  src/nmpc_ros/mpc_planner.cpp
  src/nmpc_ros/trajectory.cpp
)
add_executable(nmpc_ros
  src/nmpc_ros/mpc_planner_node.cpp
)

target_link_libraries(nmpc_ros
  ${catkin_LIBRARIES}
  nmpc_lib
  ipopt
)
add_dependencies(nmpc_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## init deceleration node
# add_library(nmpc_deceleration_lib
#   src/nmpc_deceleration/mpc_planner_ros_deceleration.cpp
#   src/nmpc_deceleration/mpc_planner_deceleration.cpp
#   src/nmpc_deceleration/trajectory.cpp
# )
# add_executable(nmpc_deceleration
#   src/nmpc_deceleration/mpc_planner_deceleration_node.cpp
# )
# target_link_libraries(nmpc_deceleration
#   ${catkin_LIBRARIES}
#   nmpc_deceleration_lib
#   ipopt
# )
# add_dependencies(nmpc_deceleration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


