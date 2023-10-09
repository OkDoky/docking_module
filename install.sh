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

## install IPOPT
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.14.4.tar.gz
tar -xvzf Ipopt-3.14.4.tar.gz 
cd Ipopt-releases-3.14.4/
apt install cppad gfortran
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps/
./get.Mumps 
./configure
make -j4 && make test && make install

cd ..
./configure --prefix=/usr/local --with-mumps-lib="-L/usr/local/lib -ldmumps -lmumps_common -lpord -lmpiseq -lesmumps -lcamseq" --with-mumps-incdir="/usr/local/include" ADD_CXXFLAGS="-I/usr/local/include"
make -j4
make test
make install

## cause cppad use coin/IpIpoptAppication.hpp
mv /usr/local/include/coin-or /usr/local/include/coin

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib'>> ~/.bashrc
soruce ~/.bashrc

# install wstool, rosdep and install & initialize dependencies packages
apt-get install -y python3-rosdep python3-wstool python3-rosinstall python3-rosinstall-generator python3-pip
pip3 install matplotlib
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
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
