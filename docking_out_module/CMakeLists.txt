cmake_minimum_required(VERSION 3.0.2)
project(docking_out_module)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  rospy
  tf
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    std_msgs
    tf
    tf2_ros
)
