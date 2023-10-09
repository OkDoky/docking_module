cmake_minimum_required(VERSION 3.0.2)
project(marker_detecting)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_msgs
  tf2_ros
  tf
  gazebo_ros
)

catkin_package()
catkin_install_python(PROGRAMS script/detecting_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
install(DIRECTORY models 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
