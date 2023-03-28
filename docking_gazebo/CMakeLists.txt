cmake_minimum_required(VERSION 3.0.2)
project(docking_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  urdf
  sensor_msgs
  tf
  tf2_ros
  gazebo_ros
  rviz
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS 
    gazebo_ros 
    tf 
    robot_state_publisher 
    joint_state_publisher 
    tf2_ros 
    geometry_msgs 
    nav_msgs 
    std_msgs 
    urdf
    rviz
  DEPENDS GAZEBO
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
