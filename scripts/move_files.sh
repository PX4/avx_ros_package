#!/bin/bash

# moves the support files to its matching directory

# Copyright (c) 2018 Auterion

INSTALL_DIR=$PWD

# bag folder needed for the calibration in kalibr

# kalibr wrapper which supports the extrinsics calibration between camera and IMU
scp rovio_footer_px4.txt nvidia@192.168.1.117:/home/nvidia/catkin_ws/src/Kalibr/aslam_offline_calibration/kalibr/python/exporters/auxiliary_files/

# patch footer needed for rovio file




# rovio launch files
cp $INSTALL_DIR/launch/rovio/.*launch $AUTERION_WS/src/rovio/launch/
mkdir -p $AUTERION_WS/src/rovio/cfg/cam_intrinsics
scp rovio_rs_infra1_D435.launch nvidia@192.168.1.117:/home/nvidia/catkin_ws/src/rovio/launch/
# realsense launch files

