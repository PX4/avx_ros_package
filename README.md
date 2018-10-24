# avx_ros_package

Flash Ubuntu image onto TX2.

Enable Universe software repositories from the Apt repository list.
```bash
sudo vim /etc/apt/sources.list
```

Uncomment the following 4 lines from the file.
```bash
deb http://ports.ubuntu.com/ubuntu-ports/ xenial universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ xenial universe
deb http://ports.ubuntu.com/ubuntu-ports/ xenial-updates universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ xenial-updates universe
```

Update package manager.
```bash
sudo apt-get update
```

Launch ubuntu_ros_mavros.sh script to install ros and mavros on the TX2.
```bash
chmod +x ubuntu_ros_mavros.sh
./ubuntu_ros_mavros.sh
```

Clone this repo inside your catkin workspace.
```bash
cd ~/catkin_ws/src
git clone https://github.com/PX4/avx_ros_package.git
```

Build the ros package using catkin.
```bash
cd ~/catkin_ws
catkin build avx_ros_package
source ~/.bashrc
```

Launch example node from avx_ros_package using the launch script. This node subscribes to local_position and state and prints in on the shell.
```bash
roslaunch avx_ros_package receiver.launch
```

Launch mavros to start recording raw imu data into a rosbag.
```bash
roslaunch avx_ros_package test.launch
```

Also the Dronecode SDK can run on the TX2 to communicate with the flight controller.

Install Dronecode SDK and takeoff_land example by launching the following script.
```bash
chmod +x ubuntu_dronecode_sdk.sh
./ubuntu_dronecode_sdk.sh
```

Launch Dronecode SDK example.
```bash
cd ~/DroneCore/example/takeoff_land/build
./takeoff_and_land
```
