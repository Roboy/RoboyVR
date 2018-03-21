Installation
=============

As described on the main page, RoboyVR is a project which spans over multiple platforms to simulate Roboy and visualize all data. Unity is used to display the virtual environment and Roboy to the user, run on a Windows OS. The virtual reality is simulated using SteamVR and a HTC Vive headset together with two hand controllers. These are tracked with a lighthouse system.

 ROS is a server on which entities can publish and subscribe to certain topics and information, hosted on an Linux / Ubuntu platform. Roboy and its behavior is simulated in Gazebo which also runs on a Linux OS and publishes its values on ROS. Unity in turn subscribes to these topics to receive all updates. In Unity roboy is rendered and constantly updated concerning positions, rotations, etc.  
With the help of a VR-Headset you can watch roboy move around in a virtual space.

This tutorial will help you setup roboyVR with all necessities it comes with.

Part 1: Setup an Ubuntu OS 
--------------------------

A machine hosting the ROS server and running the gazebo simulation is needed, conveniently, both can be combined and executed on one machine. Virtual machines were not used - it is possible, that the setup works, out of perfomance reasons and stability, a separate machine is advised. 


Part 2: Simulation Setup
-------------------------------------------

Follow the setup instructions on the main `Roboy repository <https://github.com/Roboy/Roboy>`_.

| *Note: the setup.sh of gazebo is in /usr/share/gazebo-7/setup.sh and not in ../gazebo-7.0/..*
| *Note: Export the gazebo paths AFTER the catkin_make because the devel directory is just created at this command.*
|

On top of that it may be necessary to update the submodules of this repository:

.. code:: bash
  
  cd /path-to-roboy-repository/
  git submodule update --recursive --remote
  
There may also occur an error that says that you need to install the OpenPowerlink stack library. In that case
follow the instructions on the `OpenPowerlink Homepage <http://openpowerlink.sourceforge.net/doc/2.2/2.2.0/d1/dde/page_build_stack.html>`_.
The OpenPowerlink folder lies in the *roboy_powerlink* folder.
  
Part 3: Unity Setup
-------------------

1. Download Unity
  - (latest working version with roboyVR is 2017.1.0: https://unity3d.com/de/get-unity/download/archive)

2. Install Unity
  - During the install process make sure to check also the standalone build option.  
  - Visual studio is recommended to use with Unity3D, as it is free and more user friendly than  
    MonoDevelop (standard option).
	
3. Download this project
  - Clone this github repository (master branch) to your system: https://github.com/roboy/roboyVR.git
  - Command: git clone -b master https://github.com/roboy/roboyVR.git

Part 4: Setup of the Lighthouse Tracking System
-----------------------------------------------

A lighthouse tracking system is needed to track the user's movement in space for both controllers and the headset. 
The two base station should be able to see each other clearly with no viewing obstructions in their sights. They should be put up diagonal spanning a virtual room of two by five meters. For additional information take a look at this guide `HTC Vive setup <https://www.vive.com/uk/setup/>`_.

Install Steam/SteamVR in order to be able to use the Headset in the Untiy project. 
 
Part 5: Blender & Python
------------------------

- Install the latest version of `Blender <https://www.blender.org/download/>`_

- Install the latest version of `Python <https://www.python.org/downloads/>`_

- Install the `PyXB: Python XML Schema Bindings <https://sourceforge.net/projects/pyxb/>`_

- After installation, add the Python executable directories to the environment variable PATH in order to run Python. (Windows 10: http://www.anthonydebarros.com/2015/08/16/setting-up-python-in-windows-10/)

Part 6: ZED API and Unity Plugin
--------------------------------

- Install the latest verstion of the `API <https://www.stereolabs.com/developers/>`_

- If the Unity plugin in the project is outdated then download the plugin from the same page and import it into the project to update it