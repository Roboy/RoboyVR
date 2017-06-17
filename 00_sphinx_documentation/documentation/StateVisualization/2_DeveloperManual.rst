Developer Manual
================

General 
-------

The UI design goal was to create a modular and robust UI which does not rely on continuous data input. Due to the fact, that the Virtual Reality scenes will later be merged and therefore the setup will change, it was advantageous to not create one definite UI structure already containing all the desired elements, but a modular, easy-to-adjust  base which could easily be integrated in other scenes. 

- Scripts: 
- Game objects: 
   - General game objects, which belong to the scene but not the UI, include Roboy, the background and the camera rig containing the SteamVR controllers and headset. 


UI  Implementation and Structure
--------------------------------


The UI can be structured in three basic layers: 

Front-End
_________

In this layer, objects, modes, and their respective items covering screens, labels, screen overlays and UI are contained. It serves as the frontend towards the user. The respective UI elements display the data they are given, but do not actively change or adapt to changes. 

- Scripts: 
   - DisplayScreens: This script automatically detects the number of connected displays on startup and uses the first to display the main camera on the main screen, and second camera on the latter. In case only one screen is found, it continues with the normal setup. It needs to be noted that the SteamVR glasses are not considered to be a Display by Unity. 
   
.. figure:: images/displays.*
   :align: center
   :alt: Image could not be loaded
   
   Display message
   
- Game Ojects: 
   - modes: this empty game object contains all modes the user can choose with the selection wheel. These are dis- and enabled dynamically by the script UILogic. 
   - Canvas: Each mode contains an individual canvas which is activated together with the parent (container) object. Canvas Render "Screen Space - Overlay" needs to be selected and the in-game camera belonging to the VR headset to display the Camera there.
   
.. figure:: images/canvas_setting.*
   :align: center
   :alt: Image could not be loaded
   
   Canvas settings for VR headsets
   
.. comment here: somehow this thing does not format properly if image not displayed... maybe with image its okay?. and this comment may be buggy as well

             Note that even though the canvas is stretched to fit the screen size, the display of the headset extends further than the user's view frustrum. In the picture below, the original canvas size can be seen as well as the actual view frustrum, which the user can comfortably perceive without too much strain on the eyes. 

.. figure:: images/view_frustrum.*
   :align: center
   :alt: Image could not be loaded
   
   Canvas size in blue and view frustrum in green
   
Core
____

This layer covers the UI logic. It displays the selected modes, updates the frontend based on the processed given input,  performs user requests and handles user input such as pointing, clicking and scrolling. Based on demand, it creates new UI elements, alters, updates, activates and hides these. 

- Scripts: 
   - SelectionWheelScript: This script is attached to a gameobject within a canvas, which will be disabled in the beginning. Additionally, all the children of the component are realigned to fill the selection wheel according to the number of elements. The script constantly checks for input when activated. As soon as input is detected, it enables the canvas to display the wheel and all the child objects. These are rotated on a circle according to the position of the sensed input on the controller. The controller can be set in the public variable Controllerindex. The placement on the circle, where the element should be selected, can be changed in the public variable selectionIndex. This index specifies the index within the number of game objects, which shall be selected. It starts at 12 o'clock and rotates clockwise. Since the script is general in implementation and usage, it can be used multiple times under different occasions.

   .. figure:: images/selection_wheel.*
    :align: center
    :alt: Image could not be loaded
    
    Selection wheel with four options and Overview selected
    
   - UILogic: This script operates as a database for important game values. Due to its Singleton (link: wikipedia) implementation, it is always accessible and no dublicates and therefore (versionierung / ) and all functions can use it as a data platform. It does not contain an Update() function and does not actively request data. Other functions and instances can set and get the desired data. This design choice was made, since it assures modularity of the respective elements, both front and back-end. It provides less assurance considering the age of the given data when later used.  Nevertheless, an Event-driven UI can still be implemented using the subscriber scheme. The issue there would be if the data is provided in non-continuous time intervals. One operation which is implemented using that style is the change of modes. As soon as the selected index changed, the respective game object and mode, which is linked in the public modes array,  is activated, the others are deactivated. 
- Game Objects:
   - UILogic: This empty game object is not displayed, but contains all relevant UI components as child objects. 
   
Back-End
________
As of now, there is no implementation of a back-end. 