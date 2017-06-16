Installation
=============

Roboy and its behavior is simulated on the virtual machine via ROS. Important information  
regarding roboy's movement are then sent through a ROSbridge(e.g. messages) towards Unity.  
In Unity roboy is rendered and constantly updated concerning positions, rotations, etc.  
With the help of a VR-Headset you can watch roboy move around in a virtual space.

This tutorial will help you setup roboyVR with all necessities it comes with.
 
Part 1: Setup Virtualbox with Ubuntu
--------------------------------------

1. Download and install Virtualbox for your OS https://www.virtualbox.org/

2. Download Ubuntu 16.04 (64bit) https://www.ubuntu.com/download/desktop

3. Mount the .iso and setup Virtualbox with the following settings (if available):
  a. 4 cores (Settings->System->Processor)
  b. 6 GB of RAM (Settings->System->Motherboard)
  c. 128 MB of VRAM (Settings->Display->Screen)
  d. 30 GB HDD space (Settings->Storage)

4. Set network settings to Bridged-Adapter or Host-Only Adapter

Part 2: Simulation Setup
-------------------------------------------

1. Open Terminal and install the following packages

.. code:: bash

  sudo add-apt-repository -y ppa:letrend/libcmaes
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
  sudo apt-get update
  sudo apt install libcmaes
  sudo apt-get install ros-kinetic-desktop-full
  sudo apt install ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers
  sudo apt install ros-kinetic-ecl-geometry
  sudo apt install libncurses-dev
  sudo apt-get install catkin
  sudo apt-get install git

2. Clone the git repository into a ros working space

.. code:: bash

  mkdir -p ~/ros_ws/src
  cd ~/ros_ws/src
  git clone https://github.com/Roboy/roboy-ros-control --recursive

3. Get additional dependencies

.. code:: bash

  cd roboy-ros-control
  git submodule update --init --recursive
  cd src/flexrayusbinterface
  sudo dpkg -i lib/libftd2xx_1.1.12_amd64.deb

4. Source the setup.bash

.. code:: bash

  source /opt/ros/kinetic/setup.bash
  cd ~/ros_ws
  catkin_make

5. OPTIONAL: add this to your bash script (otherwise you have to type this commands in every new terminal window)

.. code:: bash

  echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
  echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc

6. Create symlinks for gazebo to your roboy models

.. code:: bash

  cd ~
  mkdir -p ~/.gazebo/models
  ln -s ~/ros_ws/src/roboy-ros-control/src/roboy_models/legs_with_upper_body ~/.gazebo/models/

7. Install rosbridge

.. code:: bash

  sudo apt install ros-kinetic-rosbridge-suite
  
Part 3: Unity Setup
-----------------------

1. Download Unity
  - (latest working version with roboyVR is 5.6.0: https://unity3d.com/de/get-unity/download/archive)

2. Install Unity
  - During the install process make sure to check also the standalone build option.  
  - Visual studio is recommended to use with Unity3D, as it is free and more user friendly than  
    MonoDevelop (standard option).
	
3. Download this project
  - Clone this github repository (master branch) to your system: https://github.com/sheveg/roboyVR.git
  - Command: git clone -b master https://github.com/sheveg/roboyVR.git

Part 4: Blender & Python
------------------------

- Install the latest version of `Blender <https://www.blender.org/download/>`_

- Install the latest version of `Python <https://www.python.org/downloads/>`_

- After installation, add the Python executable directories to the environment variable PATH in order to run Python.
