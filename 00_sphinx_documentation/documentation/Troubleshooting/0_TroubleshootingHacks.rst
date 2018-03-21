Troubleshooting Hacks
=====================

VIVE Headset
------------


If the HTC Vive headset isn't working at all:
  - USB: Unplug the usb cable connecting the Headset with the computer you are using. Simply plug back in - in case another port is used, he drivers are newly chosen by Windows.
  - Restart SteamVR: Click the SteamVR drop-down menu, select Settings. Choose the Developer tab, scroll to the bottom and click **Reboot Vive Headset**.
  - Restart Steam: This works in case Steam tried to update SteamVR and failed.

If there are problems with tracking:

  - Make sure the lighthouses can both see each other and at least one of them can see the headset at any point in time.
  - If controllers aren't tracked make sure their batteries are charged. You can also sync them manually by right clicking on one of them in SteamVR.
  - If the tracking is off or doesn't work at all, try doing the SteamVR room setup.
  
If the position is not optimal: 
 - Open the SteamVR drop-down menu, click **Run Room Setup**. 

  
Unity
-----
Unity sometimes displays unexpected behaviour contrary to what is coded and/or expected. First advise: Restart Unity - this may sound banal, but it would have saved our team at least an hour of debugging. 
  
Simulation 
----------

1. If gazebo fails to start server or they die unexpectedly: 
	
	Kill the gazebo server and restart it.
	
	.. code:: bash
	
	  killall gzserver
	  killall gzclient
	  
	Kill Roscore
	
	.. code:: bash
	
	  killall roscore
  
2. In case Models could not be loaded - which is often the reason when simulations take forever to load: 
 
	Export the gazebo paths to the model: General Usage/Getting started/



3. If you are still having trouble, please contact the Roboy Team. We will glady help you to set up your RoboyVR experience.




