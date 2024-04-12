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






##############

echo "Script execution completed: external_git_repositories"
