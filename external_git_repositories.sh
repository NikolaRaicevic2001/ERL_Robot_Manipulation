#!/bin/bash

echo "Starting the script execution: external_git_repositories"

##########################################
############ Git Repositories ############
##########################################
cd src/
# 1) xarm_ros2
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
cd /xarm_ros2
git pull
git submodule sync
git submodule update --init --remote
cd ../
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# 2) xArm-CPLUS-SDK
git clone https://github.com/xArm-Developer/xArm-CPLUS-SDK.git
cd ./xArm-CPLUS-SDK/
make xarm
make test
make test-0002-get_property # build example/test-0002-get_property.cc
make clean
make # make xarm && make test
# ./build/example/0002-get_property 192.168.1.221
cd ../

# 3) franka_ros2
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka # only for panda
cd libfranka
git checkout 0.13.3
git submodule update
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
cpack -G DEB
sudo dpkg -i libfranka*.deb
cd ../
cd ../
git clone https://github.com/frankaemika/franka_ros2.git 

# 4) 




##############################################################

echo "Script execution completed: external_git_repositories"
