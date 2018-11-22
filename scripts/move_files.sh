#!/bin/bash

# moves the support files to its matching directory

# Copyright (c) 2018 Auterion

INSTALL_DIR=$PWD

# bag folder needed for the calibration in kalibr with bag and yaml files

mv -v $INSTALL_DIR/resources/bags/* $AUTERION_WS/bags/

# kalibr wrapper which supports the extrinsics calibration between camera and IMU
cp $INSTALL_DIR/resources/kalibr_repo/rovio_footer_px4.txt $AUTERION_WS/src/Kalibr/aslam_offline_calibration/kalibr/python/exporters/auxiliary_files/

# patch footer needed for rovio file
patch -p1 -i $INSTALL_DIR/patches/kalibr_rovio_config.patch



# rovio launch files
cp $INSTALL_DIR/launch/rovio/* $AUTERION_WS/src/rovio/launch/
#mkdir -p $AUTERION_WS/src/rovio/cfg/cam_intrinsics
cp -r $INSTALL_DIR/resources/rovio/* $AUTERION_WS/src/rovio/cfg/
# realsense launch files

