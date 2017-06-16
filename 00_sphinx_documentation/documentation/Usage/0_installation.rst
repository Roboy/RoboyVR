Installation
=============

Roboy and its behavior is simulated on the virtual machine via ROS. Important information  
regarding roboy's movement are then sent through a ROSBridge(e.g. messages) towards Unity.  
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

Follow the setup instructions on the main `Roboy repository <https://github.com/Roboy/Roboy>`_.

| *Note: the setup.sh of gazebo is in /usr/share/gazebo-7/setup.sh and not in ../gazebo-7.0/..*
| *Note: Export the gazebo paths AFTER the catkin_make because the devel directory is just created at this command.*

There may also occur an error that says that you need to install the OpenPowerlink stack library. In that case
follow the instructions on the `OpenPowerlink Homepage <http://openpowerlink.sourceforge.net/doc/2.2/2.2.0/d1/dde/page_build_stack.html>`_.
The OpenPowerlink folder lies in the *roboy_powerlink* folder.
  
Part 3: Unity Setup
-----------------------

1. Download Unity
  - (latest working version with roboyVR is 5.6.1: https://unity3d.com/de/get-unity/download/archive)

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
