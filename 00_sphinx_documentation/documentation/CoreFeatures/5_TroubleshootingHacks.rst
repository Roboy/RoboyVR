Troubleshooting Hacks
=====================

If the HTC Vive headset isn't working at all:

  - Simple USB-Trick: Unplug the usb cable connecting the htc box with the pc you are using. Simply plug it in the same port. Restart SteamVr.
  - Advanced USB-Trick: Unplug everything from the htc box and using another USB port plug all cables in, this makes it so drivers are installed again.
  - Restart the HTC Vive through the SteamVR settings (developer).

If there are problems with tracking:

  - Make sure the lighthouses can both see each other and at least one of them can see the headset at any point in time.
  - If controllers aren't tracked make sure their batteries are charged. You can also sync them manually by right clicking on one of them in SteamVR.
  - If the tracking is off or doesn't work at all, try doing the SteamVR room setup.
  
  
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




