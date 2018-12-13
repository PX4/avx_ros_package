#!/bin/bash

# builds realsense ROS wrapper, rovio and kalibr

# Copyright (c) 2018 Auterion


#create workspace
cd ${HOME}
ws_name=catkin_ws
mkdir -p ${HOME}/$ws_name/src
NEW_WS=${HOME}/$ws_name
REALSENSE_DIRECTORY=$NEW_WS/src/realsense
ROVIO_DIR=$NEW_WS/src/rovio
green=`tput setaf 2`
reset=`tput sgr0`

cd $NEW_WS
catkin clean
echo $PWD
catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
	-DCATKIN_ENABLE_TESTING=False \
	&& catkin config --extend /opt/ros/kinetic \
	&& catkin config --merge-devel

## Initialize rosdep
rosdep init && rosdep update

## Setup environment variables
rossource="source /opt/ros/kinetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource
## Get rosinstall
sudo apt-get install python-rosinstall -y
cd src/

# install realsense ROS Wrapper
echo "${green}Cloning ROS Realsense${reset}"
git clone https://github.com/Auterion/realsense.git
cd $NEW_WS

cd $REALSENSE_DIRECTORY
git fetch --all
git checkout 255805851761d8565bf6c61d174a2db125b31a29
catkin build realsense2_camera --pre-clean

## Re-source environment to reflect new packages/build environment
catkin_ws_source="source $NEW_WS/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc;
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source

# install kalibr
sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-dev libboost-all-dev doxygen \
 libopencv-dev ros-kinetic-vision-opencv ros-kinetic-image-transport-plugins ros-kinetic-cmake-modules \
 python-software-properties software-properties-common libpoco-dev python-matplotlib python-scipy \
 python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev python-catkin-tools libv4l-dev python-igraph\
 python-pandas

sudo pip install python-igraph --upgrade

cd $NEW_WS/src/
echo "${green}Cloning Kalibr${reset}"
git clone https://github.com/ethz-asl/Kalibr.git
cd $NEW_WS
catkin build kalibr -j4
$catkin_ws_source



# install caktin dependencies
sudo apt-get install cmake python-catkin-pkg python-empy python-nose libgtest-dev
cd $NEW_WS/src
git clone https://github.com/ros/catkin.git
git clone https://github.com/ethz-asl/yaml_cpp_catkin.git

#install rovio
echo "${green}Cloning rovio${reset}"
git clone https://github.com/ethz-asl/rovio.git
cd $ROVIO_DIR
sudo apt-get install freeglut3-dev  libglew-dev
git submodule update --init --recursive

cd $NEW_WS/src
git clone https://github.com/ANYbotics/kindr.git
catkin build -w $NEW_WS kindr
$catkin_ws_source


catkin build rovio --cmake-args  -DMAKE_SCENE=OFF
source $NEW_WS/devel/setup.bash
cd $NEW_WS
export AUTERION_WS=$(pwd)
echo 'export AUTERION_WS='$(pwd) >> ~/.bashrc
mkdir bags
source ~/.bashrc
echo ""
echo ""

cd 
cd .config 
mkdir matplotlib
echo "backend: Agg" > ~/.config/matplotlib/matplotlibrc


