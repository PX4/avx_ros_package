#!/bin/bash

# moves the support files to its matching directory

# Copyright (c) 2018 Auterion

INSTALL_DIR=$PWD

# bag folder needed for the calibration in kalibr with bag and yaml files

mv -v $INSTALL_DIR/resources/bags/* $AUTERION_WS/bags/
cd $AUTERION_WS/bags/
wget --no-check-certificate 'https://drive.google.com/uc?id=14gEK33zrKNFywN4LOB_XKK8ys_U3yU0t&export=download' -O auterion_validation.bag.tar.gz
tar -zxvf auterion_validation.bag.tar.gz 
rm auterion_validation.bag.tar.gz 



# kalibr wrapper which supports the extrinsics calibration between camera and IMU
cp $INSTALL_DIR/resources/kalibr_repo/rovio_footer_px4.txt $AUTERION_WS/src/Kalibr/aslam_offline_calibration/kalibr/python/exporters/auxiliary_files/

# patch the rovio exporter in kalibr
cd $AUTERION_WS/src/Kalibr/
patch -p1 -i $INSTALL_DIR/patches/kalibr_rovio_config.patch

#patch the image queue in rovio
cd $AUTERION_WS/src/rovio/
patch -p1 -i $INSTALL_DIR/patches/RovioNode.patch

cd cd $AUTERION_WS
catkin build rovio --cmake-args  -DMAKE_SCENE=OFF
source $AUTERION_WS/devel/setup.bash


# rovio launch files
cp $INSTALL_DIR/launch/rovio/* $AUTERION_WS/src/rovio/launch/
#mkdir -p $AUTERION_WS/src/rovio/cfg/cam_intrinsics
cp -r $INSTALL_DIR/resources/rovio/* $AUTERION_WS/src/rovio/cfg/
# realsense launch files

