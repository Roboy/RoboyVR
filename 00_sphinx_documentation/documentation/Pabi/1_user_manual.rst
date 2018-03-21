User's Manual
=============

Setup Ubuntu side
-----------------

As you already installed gazebo and the roboy project like described in the installation part you need only to start the *.launch* file.

1. Source the setup.bash

.. code:: bash

  source /path-to-roboy-ros-control/devel/setup.bash

2. Start the launch file which starts Gazebo with the PaBi legs and a PaBiDanceSimulator ROS node

.. code:: bash

  roslaunch roboy_simulation pabi_world.launch
  
This should be the result:

.. figure:: images/pabi_simulation_dance.*
	:align: center
	:alt: PaBi Dance
         
	PaBi model in Gazebo

Setup Unity side
----------------

You should have the RoboyVR project already cloned on your local machine. Therefore you only need to start Unity and open the PaBiViveScene.
There should be a ROSBridge object in the hierarchy. Select this object and enter the IP Adress of the machine on which the simulation is running.

.. figure:: ../images/rosbridge.*
	:align: center
	:alt: ROSBridge
         
	ROSBridge in Unity


As soon as you start the scene SteamVR should open if that is not already the case. Then you have to follow the instructions on the screen to setup your Vive controllers. 

.. figure:: ../images/controller_selection_1.*
    :align: center
    :alt: Controller Selection GUI
         
	Instruction to choose the GUI Controller


.. figure:: ../images/controller_selection_2.*
    :align: center
    :alt: Controller Selection Tools
         
	Instruction to choose the Tool Controller


Afterwards you can watch PaBi showing his best dance moves and interact with him via a GUI and different tools.

.. figure:: images/pabi_selection.*
    :align: center
    :alt: PaBi Selection
         
	Unity Scene of PaBi


*Note: Shooting PaBi with the nerf gun does not have any consequences*