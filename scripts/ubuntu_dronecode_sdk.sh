#!/bin/bash

## Bash script for setting up a ROS/Gazebo development environment for PX4 on Ubuntu LTS (16.04). 
##
## Installs:
## - Dependencies for Dronecore 
## - Dronecore libraries

## Install dependencies
sudo apt-get update -y
sudo apt-get install cmake build-essential colordiff astyle git libcurl4-openssl-dev libtinyxml2-dev doxygen -y

## clone dronecore repository
cd ~
git clone https://github.com/dronecore/DroneCore.git
cd DroneCore
git submodule update --init --recursive

## build and install dronecore library
make default
sudo make default install
sudo ldconfig  			

## build takeoff_and_land example
cd example/takeoff_land/
mkdir build && cd build
cmake ..
make

## launch example with the following command
# ./takeoff_and_land




