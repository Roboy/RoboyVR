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

.. figure:: ../rosbridge.*
    :align: center
    :alt: ROS Bridge

4. Start the scene.
5. SteamVR should also start, if this throws errors (like "SteamVR unresponsive, not working, etc."), simply restart it.
6. When the scene properly starts, you can choose which controller should hold which tools.
7. After the controller assignment, you can switch between various view modes via a selection menu in the scene.
8. Enjoy your stay!


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

