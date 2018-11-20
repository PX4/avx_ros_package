#!/bin/bash

# moves the support files to its matching directory

# Copyright (c) 2018 Auterion

INSTALL_DIR=$PWD

# kalibr wrapper which supports the extrinsics calibration between camera and IMU

# bag folder needed for the calibration in kalibr



# rovio launch files
cp $INSTALL_DIR/launch/rovio/.*launch $AUTERION_WS/src/rovio/launch/
# realsense launch files

