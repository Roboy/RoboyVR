User's Manual
=============

**NOTICE: THIS CODE IS NOT BEING MAINTAINED AND MAY THEREFORE NOT WORK CORRECTLY ANYMORE OR THE INFORMATION MAY BE OUTDATED**

Setup Ubuntu side
-----------------

Assuming Gazebo is already installed as described in the *Installation* and *Getting Started* sections, the *.launch* file needs to be run.

1. Source/ export all paths and files as initially described in the *Getting Started* section.

2. Run the launch file which starts ROS, Gazebo with the PaBi legs and a PaBiDanceSimulator ROS node

**NOTICE: THIS COMMAND IS NO LONGER WORKING AS HTHE FILES WHERE RENAMED. CHOOSE THE APPROPRIATE .LAUNCH FILE **

.. code:: bash

  roslaunch roboy_simulation pabi_world.launch
  
This should be the result:

.. figure:: images/pabi_simulation_dance.*
	:align: center
	:alt: PaBi Dance
         
	PaBi model in Gazebo

Setup Unity side
----------------

Unity should be set up according to the *Installation* and *Getting Started* instructions. Therefore, only start Unity and open the PaBiViveScene.
There should be a ROSBridge object in the hierarchy. Select this object and enter the IP Adress of the machine on which the simulation is running. PaBi now shows the dance moves, interaction with the leg model is possible via a GUI and different tools.

.. figure:: images/pabi_selection.*
    :align: center
    :alt: PaBi Selection
         
	Unity Scene of PaBi


*Note: Shooting PaBi with the nerf gun does not have any consequences*