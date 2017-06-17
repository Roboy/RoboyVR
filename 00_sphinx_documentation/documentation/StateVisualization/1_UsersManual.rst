User's Manual
=============
  
Set-up
------

The Scene can be run in the Unity Editor. Simply double click on the UIScene, Unity will start and load the scene. The play button in the top-centre starts it. The scene works with and without connected SteamVR headset and controller, though it does not change the camera or control panels without these interaction methods, as these are the single input method. The Screen is displayed in a window in Unity and in the glasses. Since as of now there is no connection to ROS, this aspect does not need to be considered. Both the play buttons as well as the game window can be seen highlighted in the screenshot below. 

.. figure:: images/unity_editor.*
    :align: center
    :alt: Unity Editor
    
    
SteamVR
-------

For further information on how to set up the VR headset and controllers, see TODO .

UI Features
-----------

The user can initially decide, which controller is his main controller by using the trigger at the underside of the controller. The main controller touchpad can then be used to change between different modes (Cognition, Overview, Middleware and Control). By turning the head with the headset, the camera can be rotated. By moving around the room, the camera position can be changed. Beware of possible obstacles and boundaries given by the physical surrounding. 

With the raycaster - seen as a red beam -  different body parts of Roboy can be selected and information can be displayed. This is the current state of implementation, further changes and updates will be made in the future. 