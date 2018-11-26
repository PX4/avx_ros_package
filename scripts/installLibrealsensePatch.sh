#!/bin/bash
# Builds the Intel Realsense library librealsense on a Jetson TX Development Kit
# Copyright (c) 2018 Auterion 
# MIT License

# librealsense requires CMake 3.8+ to build; 

LIBREALSENSE_DIRECTORY=${HOME}/librealsense
LIBREALSENSE_VERSION=v2.16.0


JETSONHACKS_DIRECTORY=${HOME}/buildLibrealsense2TX
cd ..
INSTALLATION_DIRECTORY=$PWD


# get repo with patches from jetsonhacks
cd ${HOME}
git clone https://github.com/jetsonhacks/buildLibrealsense2TX.git
cd $JETSONHACKS_DIRECTORY


#BUILD_CMAKE=false

function usage
{
    echo "usage: ./installLibrealsensePatch.sh [[-c ] | [-h]]"

    echo "-h | --help  This message"
}

# Iterate through command line inputs
while [ "$1" != "" ]; do
    case $1 in
        #-n | --no_cmake )      shift
				#BUILD_CMAKE=false
        #                        ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

red=`tput setaf 1`
green=`tput setaf 2`
reset=`tput sgr0`
# e.g. echo "${red}The red tail hawk ${green}loves the green grass${reset}"


echo ""
echo "Please make sure that no RealSense cameras are currently attached"
echo ""
read -n 1 -s -r -p "Press any key to continue"
echo ""

if [ ! -d "$LIBREALSENSE_DIRECTORY" ] ; then
  # clone librealsense
  cd ${HOME}
  echo "${green}Cloning librealsense${reset}"
  git clone https://github.com/IntelRealSense/librealsense.git
fi

# Is the version of librealsense current enough?
cd $LIBREALSENSE_DIRECTORY
VERSION_TAG=$(git tag -l $LIBREALSENSE_VERSION)
if [ ! $VERSION_TAG  ] ; then
   echo ""
  tput setaf 1
  echo "==== librealsense Version Mismatch! ============="
  tput sgr0
  echo ""
  echo "The installed version of librealsense is not current enough for these scripts."
  echo "This script needs librealsense tag version: "$LIBREALSENSE_VERSION "but it is not available."
  echo "This script patches librealsense, the patches apply on the expected version."
  echo "Please upgrade librealsense before attempting to install again."
  echo ""
  exit 1
fi

# Checkout version the last tested version of librealsense
git checkout $LIBREALSENSE_VERSION

# Install the dependencies
cd $JETSONHACKS_DIRECTORY # changed here
sudo ./scripts/installDependencies.sh

# Do we need to install CMake no 
# if [ "$BUILD_CMAKE" = true ] ; then
#   echo "Building CMake"
#   ./scripts/buildCMake.sh
#   CMAKE_BUILD_OK=$?
#   if [ $CMAKE_BUILD_OK -ne 0 ] ; then
#     echo "CMake build failure. Exiting"
#     exit 1
#   fi
# fi

cd $LIBREALSENSE_DIRECTORY
git checkout $LIBREALSENSE_VERSION

echo "${green}Applying Model-Views Patch${reset}"
# The render loop of the post processing does not yield; add a sleep
patch -p1 -i $JETSONHACKS_DIRECTORY/patches/model-views.patch

echo "${green}Applying Incomplete Frames Patch${reset}"
# The Jetson tends to return incomplete frames at high frame rates; suppress error logging
patch -p1 -i $INSTALLATION_DIRECTORY/patches/incomplete_frame.patch


echo "${green}Applying udev rules${reset}"
# Copy over the udev rules so that camera can be run from user space
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

# Now compile librealsense and install
mkdir build 
cd build
# Build examples, including graphical ones
echo "${green}Configuring Make system${reset}"
# Use the CMake version that we built, must be > 3.8
cd $LIBREALSENSE_DIRECTORY/build
cmake ../ -DBUILD_EXAMPLES=true  
# The library will be installed in /usr/local/lib, header files in /usr/local/include
# The demos, tutorials and tests will located in /usr/local/bin.
echo "${green}Building librealsense, headers, tools and demos${reset}"

NUM_CPU=$(nproc)
time make -j$(($NUM_CPU - 1))
if [ $? -eq 0 ] ; then
  echo "librealsense make successful"
else
  # Try to make again; Sometimes there are issues with the build
  # because of lack of resources or concurrency issues
  echo "librealsense did not build " >&2
  echo "Retrying ... "
  # Single thread this time
  time make 
  if [ $? -eq 0 ] ; then
    echo "librealsense make successful"
  else
    # Try to make again
    echo "librealsense did not successfully build" >&2
    echo "Please fix issues and retry build"
    exit 1
  fi
fi
echo "${green}Installing librealsense, headers, tools and demos${reset}"
sudo make install
echo "${green}Library Installed${reset}"
echo " "
echo " -----------------------------------------"
echo "The library is installed in /usr/local/lib"
echo "The header files are in /usr/local/include"
echo "The demos and tools are located in /usr/local/bin"
echo " "
echo " -----------------------------------------"
echo " "



CLEANUP=false


cd $JETSONHACKS_DIRECTORY
echo " "
echo " -----------------------------------------"
echo "${green}Beginning building a new kernel${reset}"
echo "${green}Please don't disrupt while building${reset}."
echo " "
echo " -----------------------------------------"
echo " "
# Is this the correct kernel version?
source scripts/jetson_variables.sh
#Print Jetson version
echo "$JETSON_DESCRIPTION"
#Print Jetpack version
echo "Jetpack $JETSON_JETPACK [L4T $JETSON_L4T]"
echo "Jetson $JETSON_BOARD Development Kit"

# Error out if something goes wrong
set -e

# Check to make sure we're installing the correct kernel sources
# Determine the correct kernel version
# The KERNEL_BUILD_VERSION is the release tag for the JetsonHacks buildKernel repository
KERNEL_BUILD_VERSION=master
if [ $JETSON_BOARD == "TX2" ] ; then 
L4TTarget="28.2.1"
  # Test for 28.2.1 first
  if [ $JETSON_L4T = "28.2.1" ] ; then
     KERNEL_BUILD_VERSION=vL4T28.2.1
  elif [ $JETSON_L4T = "28.2" ] ; then
     KERNEL_BUILD_VERSION=vL4T28.2r3
  else
   echo ""
   tput setaf 1
   echo "==== L4T Kernel Version Mismatch! ============="
   tput sgr0
   echo ""
   echo "This repository is for modifying the kernel for a L4T "$L4TTarget "system." 
   echo "You are attempting to modify a L4T "$JETSON_L4T "system."
   echo "The L4T releases must match!"
   echo ""
   echo "There may be versions in the tag/release sections that meet your needs"
   echo ""
   exit 1
  fi 
fi

if [ $JETSON_BOARD == "TX1" ] ; then 
 L4TTarget="28.2"
 if [ $JETSON_L4T = "28.2" ] ; then
     KERNEL_BUILD_VERSION=v1.0-L4T28.2
  else
   echo ""
   tput setaf 1
   echo "==== L4T Kernel Version Mismatch! ============="
   tput sgr0
   echo ""
   echo "This repository is for modifying the kernel for L4T "$L4TTarget "system." 
   echo "You are attempting to modify a L4T "$JETSON_L4T "system."
   echo "The L4T releases must match!"
   echo ""
   echo "There may be versions in the tag/release sections that meet your needs"
   echo ""
   exit 1
  fi
fi

# If we didn't find a correctly configured TX2 or TX1 exit, we don't know what to do
if [ $KERNEL_BUILD_VERSION = "master" ] ; then
   tput setaf 1
   echo "==== L4T Kernel Version Mismatch! ============="
   tput sgr0
    echo "Currently this script works for the Jetson TX2 and Jetson TX1."
   echo "This processor appears to be a Jetson $JETSON_BOARD, which does not have a corresponding script"
   echo ""
   echo "Exiting"
   exit 1
fi

# Is librealsense on the device?

if [ ! -d "$LIBREALSENSE_DIRECTORY" ] ; then
   echo "The librealsense repository directory is not available"
   read -p "Would you like to git clone librealsense? (y/n) " answer
   case ${answer:0:1} in
     y|Y )
         # clone librealsense
         cd ${HOME}
         echo "${green}Cloning librealsense${reset}"
         git clone https://github.com/IntelRealSense/librealsense.git
         cd librealsense
         # Checkout version the last tested version of librealsense
         git checkout $LIBREALSENSE_VERSION
     ;;
     * )
         echo "Kernel patch and build not started"   
         exit 1
     ;;
   esac
fi

# Is the version of librealsense current enough?
cd $LIBREALSENSE_DIRECTORY
VERSION_TAG=$(git tag -l $LIBREALSENSE_VERSION)
if [ ! $VERSION_TAG  ] ; then
   echo ""
  tput setaf 1
  echo "==== librealsense Version Mismatch! ============="
  tput sgr0
  echo ""
  echo "The installed version of librealsense is not current enough for these scripts."
  echo "This script needs librealsense tag version: "$LIBREALSENSE_VERSION "but it is not available."
  echo "This script uses patches from librealsense on the kernel source."
  echo "Please upgrade librealsense before attempting to patch and build the kernel again."
  echo ""
  exit 1
fi

KERNEL_BUILD_DIR=""
cd $JETSONHACKS_DIRECTORY
echo "Ready to patch and build kernel "$JETSON_BOARD
if [ $JETSON_BOARD == "TX2" ] ; then 
  git clone https://github.com/jetsonhacks/buildJetsonTX2Kernel.git
  KERNEL_BUILD_DIR=buildJetsonTX2Kernel
  cd $KERNEL_BUILD_DIR
  git checkout $KERNEL_BUILD_VERSION
elif [ $JETSON_BOARD == "TX1" ] ; then
    git clone https://github.com/jetsonhacks/buildJetsonTX1Kernel.git
    KERNEL_BUILD_DIR=buildJetsonTX1Kernel
    cd $KERNEL_BUILD_DIR
    git checkout $KERNEL_BUILD_VERSION
  else 
    tput setaf 1
    echo "==== Build Issue! ============="
    tput sgr0
    echo "There are no build scripts for this Jetson board"
    exit 1
fi

# Get the kernel sources; does not open up editor on .config file
echo "${green}Getting Kernel sources${reset}"
./getKernelSourcesNoGUI.sh
cd ..
echo "${green}Patching and configuring kernel${reset}"
sudo ./scripts/configureKernel.sh
sudo ./scripts/patchKernel.sh

cd $KERNEL_BUILD_DIR
# Make the new Image and build the modules
echo "${green}Building Kernel and Modules${reset}"
./makeKernel.sh
# Now copy over the built image
./copyImage.sh
# Remove buildJetson Kernel scripts
if [ $CLEANUP == true ]
then
 echo "Removing Kernel build sources"
 ./removeAllKernelSources.sh
 cd ..
 sudo rm -r $KERNEL_BUILD_DIR
else
 echo "Kernel sources are in /usr/src"
fi


echo "${green}Please reboot for changes to take effect${reset}"

