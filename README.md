# Setup roboyVR

This tutorial will help you setup roboyVR.

## Part 1.1: Setup Virtualbox with Ubuntu

1) Download and install Virtualbox for your OS https://www.virtualbox.org/

2) Download Ubuntu 16.04 (64bit) https://www.ubuntu.com/download/desktop

3) Mount the .iso and setup Virtualbox with the following settings (if available):

* 4 cores
* 6 GB of RAM
* 128 MB of VRAM
* 30 GB HDD space

4) Set network settings to Bridged-Adapter or Host-Only

## Part 1.2: Install ROS & Roboy on Ubuntu

1) Open Terminal and install the following packages

```
#!bash
sudo add-apt-repository -y ppa:letrend/libcmaes
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt install libcmaes
sudo apt-get install ros-kinetic-desktop-full
sudo apt install ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers
sudo apt install libncurses-dev
sudo apt-get install catkin
sudo apt-get install git
```

2) Clone the git repository into a ros working space

```
#!bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/Roboy/roboy-ros-control --recursive
```

3) Source the setup.bash

```
source /opt/ros/kinetic/setup.bash
cd ~/ros_ws
catkin_make
```

4) OPTIONAL: add this to your bash script (otherwise you have to type this commands in every new terminal window)

```
echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
echo 'source ~/ros_ws/src/roboy-ros-control/devel/setup.bash' >> ~/.bashrc
```

5) Create symlinks for gazebo to your roboy models

```
#!bash
cd ~
mkdir -p ~/.gazebo/models
ln -s /ros_ws/src/roboy-ros-control/src/roboy_models/legs_with_upper_body ~/.gazebo/models/
```

6) Install rosbridge

```
#!bash
sudo apt install ros-kinetic-rosbridge-suite
```

7) Run rosbridge and roboySimulation

```
#!bash
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun roboy_simulation VRRoboy
```
