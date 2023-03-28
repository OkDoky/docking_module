#! /bin/bash

apt update && apt upgrade -y
name_ros_version=${name_ros_version:="noetic"}

# install default dependencies
apt install -y git apt-utils build-essential 

# install cpp-11 version
apt install -y software-properties-common
add-apt-repository ppa:ubuntu-toolchain-r/test
apt-get update
apt-get install -y gcc-11 g++-11
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 60 --slave /usr/bin/g++ g++ /usr/bin/g++-11

source /opt/ros/${name_ros_version}/setup.bash

# init workspace of source
mkdir -p /root/catkin_ws/src
cd ~/catkin_ws/src && catkin_init_workspace
git clone https://github.com/OkDoky/docking_module.git -b master
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b noetic-devel
git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git -b ros1

# install rosdep and initialize rosdep
cd ~/catkin_ws
apt install -y python3-rosdep python3-wstool && rosdep init && rosdep update

# setup bashrc
echo 'alias cw="cd ~/catkin_ws"' >> ~/.bashrc
echo 'alias cs="cd ~/catkin_ws/src"' >> ~/.bashrc
echo 'alias cm="cd ~/catkin_ws && catkin_make"' >> ~/.bashrc
echo 'alias sb="source ~/.bashrc"' >> ~/.bashrc
echo 'alias eb="nano ~/.bashrc"' >> ~/.bashrc
echo 'alias simul="roslaunch docking_simul simul.launch"' >> ~/.bashrc

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
echo 'export ROS_MASTER_URI=http://localhost:11311' >> ~/.bashrc

source ~/.bashrc && cm
