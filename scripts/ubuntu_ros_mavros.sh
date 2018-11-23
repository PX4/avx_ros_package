#!/bin/bash

## Bash script for setting up a ROS/Gazebo development environment for PX4 on Ubuntu LTS (16.04). 
## It installs the common dependencies for all targets (including Qt Creator) and the ROS Kinetic/Gazebo 7 (the default).
##
## Installs:
## - ROS Kinetic 
## - MAVROS

## ROS Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
sudo apt-get install ros-kinetic-desktop-full -y
## Specific for TX2
sudo c_rehash /etc/ssl/certs
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/kinetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource
## Get rosinstall
sudo apt-get install python-rosinstall -y

# MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
## Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

## Install dependencies
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

## Initialise wstool
wstool init ~/catkin_ws/src

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
### Get latest released mavlink package
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
if ! rosdep install --from-paths src --ignore-src --rosdistro kinetic -y; then
    # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
    unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial
fi
## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc; 
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source

## install geographiclib
cd ~/catkin_ws
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

if [[ ! -z $unsupported_os ]]; then
    >&2 echo -e "\033[31mYour OS ($unsupported_os) is unsupported. Assumed an Ubuntu 16.04 installation,"
    >&2 echo -e "and continued with the installation, but if things are not working as"
    >&2 echo -e "expected you have been warned."
fi

## install mavlink-router
cd 
git clone https://github.com/intel/mavlink-router.git
cd mavlink-router
git submodule update --init --recursive
 ./autogen.sh && ./configure CFLAGS='-g -O2' \
        --sysconfdir=/etc --localstatedir=/var --libdir=/usr/lib64 \
    --prefix=/usr

make 
sudo make install

#change config file
cd /etc
sudo mkdir mavlink-router
cd mavlink-router
sudo touch main.conf

#create config file

echo "
[General]
#Mavlink-router serves on this TCP port
TcpServerPort=5790
ReportStats=false
MavlinkDialect=auto


[UartEndpoint px4]
Device = /dev/ttyTHS1
Baud = 500000

[UdpEndpoint mavros]
Mode = Normal
Address = 127.0.0.1
Port = 14540

[UdpEndpoint UserRemote]
Mode = Normal
Address = 192.168.1.XXX
Port = 14550
" > main.conf

#add file in systemd 
cd /etc/systemd/system
sudo touch mavlink-router.service

echo "" > mavlink-router.service
