User's Manual
=============
  
Set-up
------

The Scene can be run in the Unity Editor. Simply double click on the UIScene, Unity will start and load the scene. The play button in the top-centre starts it. The scene works with and without the connected SteamVR headset and controller, though it does not change the camera or control panels without these interaction methods, as these are the only input method. The Screen is displayed in a window in Unity and in the glasses. Since there is no connection to ROS so far, this aspect does not need to be considered. Both the play buttons as well as the game window can be seen highlighted in the screenshot below. 

.. figure:: images/unity_editor.*
    :align: center
    :alt: Image could not be loaded
    
    Unity Editor
    
SteamVR
-------

The hardware of the computer needs to support Virtual Reality applications, additionally SteamVR needs to be installed. For further information on how to set up the VR headset and controller, follow the instructions.


UI Features
-----------

The user interface as of now implements different modes focussing on certain data sets. 
- The Overview provides general information about Roboy, a graph plotting a heartbeat-like function and displays notifications concerning the state of roboy. These relate to a cretain body part, which is highlighted using a halo and a warning sign near the respective area. Different types include error, warning, debug and information notifications. 
- The Middleware mode visualizes tendons as colored lines which change color depending on the applied force and update their positions depending on the movement of the robot. 
- In Control mode, additional information about notifications is given. A list provides all currently detected status messages, which display additional information when clicked. 
- The cognition mode does not contain any visualizations of specific data types. Only template screens are provided in this sceen. 


The user can initially decide, which controller is his main controller by using the trigger at the underside of the controller. Controllers are used to help the user navigate and control the environment as well as interact with the data sets. A selection wheel is used to change between modes, all other interaction methods only include point-and-click and click-and-hold using a raycaster displazing a red beam. Not all UI elements are interactive just yet, but this is to be changed in future updates. 
The main controller touchpad can then be used to change between different modes (Cognition, Overview, Middleware and Control). By turning the head with the headset, the camera can be rotated. By moving around the room, the camera position can be changed. Beware of possible obstacles and boundaries given by the physical surroundings. 

. 