User's Manual
=============

**Setup Ubuntu side**

As you already installed gazebo and the roboy project like described in the installation part you need only to start the *.launch* file.

1. Source the setup.bash

.. code:: bash

  source /path-to-roboy-ros-control/devel.setup.bash

2. Start the launch file which starts Gazebo with the PaBi legs and a PaBiDanceSimulator ROS node

.. code:: bash

  roslaunch roboy_simulation pabi_world.launch

**Troubleshooting**

This commands should be sufficient but it can happen that gazebo has problems loading the PaBi Model into the world or starting the gazebo server.

1. Kill the gazebo server and restart it.

.. code:: bash

  killall gzserver
  killall gzclient

2. Export the gazebo paths to the model

.. code:: bash

  source /usr/share/gazebo-7/setup.sh
  export GAZEBO_MODEL_PATH=/path/to/roboy-ros-control/src/roboy_models:$GAZEBO_MODEL_PATH

3. If nothing helps than write an email to roboyvr@gmail.com. We will glady help you to experience the RoboyVR-Experience.
