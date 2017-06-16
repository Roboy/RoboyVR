User's Manual
=============
This manual will descripe the steps required to start BeRoboy™ and begin your journey.

Starting Gazebo
---------------
Start your Ubuntu machine and open a terminal.

1. Source the setup.bash

.. code:: bash

  source /path-to-roboy-ros-control/devel.setup.bash
  
2. Start the launch file which starts Gazebo with the Roboy and a Camera ROS node

.. code:: bash

  roslaunch roboy_simulation camera_test.launch  


Starting Unity
--------------
1. Start the unity project inside the git repo you cloned to your hard drive.
2. Inside unity select the RoboyVR scene.
3. In the ROSBridge (located in the hierarchy) type in the correct IP of your Ubuntu machine.

.. figure:: images/rosbridge.*
    :align: center
    :alt: ROS Bridge

4. Start the scene.
5. SteamVR should also start, if this throws errors (like "SteamVR unresponsive, not working, etc."), simply restart it.

.. figure:: images/steamvr_error.*
    :align: center
    :alt: SteamVR Error

6. When the scene properly starts, you can choose which controller should hold which tools.

.. figure:: images/controller_selection_2.*
    :align: center
    :alt: Controller Selection

7. After the controller assignment, you can switch between various view modes via a selection menu in the scene.

.. figure:: images/view_selection_pointing.*
    :align: center
    :alt: View Selection

8. Enjoy your stay!


View Scenarios
--------------
You can choose between the following four view scenarios, each of the offering different things to explore!

.. figure:: images/beroboy_simulation.*
    :align: center
    :alt: Simulation view
	
	Take control over the simulation Roboy and see what he does in gazebo.

.. figure:: images/beroboy_zed.*
    :align: center
    :alt: ZED view
	
	Look through the eyes of the real Roboy and control him in real life.

.. figure:: images/beroboy_observer.*
    :align: center
    :alt: Observer view
	
	Sit back, relax, take a look at Roboy from a safe distance and watch him do some stuff.

.. figure:: images/beroboy_vr.*
    :align: center
    :alt: VR view
	
	Slip into the role of the true VR Roboy, cause mayhem or look cute, you decide.



Troubleshooting
---------------

If gazebo encounters problems loading the Model into the world or starting the server,
these commands could be useful.

1. Kill the gazebo server and restart it.

.. code:: bash

  killall gzserver
  killall gzclient

2. Export the gazebo paths to the model

.. code:: bash

  source /usr/share/gazebo-7/setup.sh
  export GAZEBO_MODEL_PATH=/path/to/roboy-ros-control/src/roboy_models:$GAZEBO_MODEL_PATH

3. If you are still having trouble, please contact roboyvr@gmail.com.
We will glady help you to enjoy your RoboyVR experience.

