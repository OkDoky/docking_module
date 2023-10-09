#! /bin/bash

apt-get update && apt-get upgrade -y
name_ros_version=${name_ros_version:="noetic"}

# install default dependencies
apt-get install -y git apt-utils build-essential software-properties-common

source /opt/ros/${name_ros_version}/setup.bash

# init workspace of source
mkdir -p /root/catkin_ws/src
cd ~/catkin_ws/src && catkin_init_workspace
git clone https://github.com/OkDoky/docking_module.git -b master

# install wstool, rosdep and install & initialize dependencies packages
apt-get install -y python3-rosdep python3-wstool python3-rosinstall python3-rosinstall-generator
cd ~/catkin_ws/src
wstool init
wstool merge ./docking_module/.rosinstall
wstool update
cd ~/catkin_ws
rosdep init && rosdep update && rosdep install -y -r --from-paths src --ignore-src

# setup bashrc
echo 'alias cw="cd ~/catkin_ws"' >> ~/.bashrc
echo 'alias cs="cd ~/catkin_ws/src"' >> ~/.bashrc
echo 'alias cm="cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release"' >> ~/.bashrc
echo 'alias sb="source ~/.bashrc"' >> ~/.bashrc
echo 'alias eb="nano ~/.bashrc"' >> ~/.bashrc
echo 'alias simul="roslaunch docking_gazebo simul.launch"' >> ~/.bashrc

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
echo 'export ROS_MASTER_URI=http://localhost:11311' >> ~/.bashrc

source ~/.bashrc
# cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
