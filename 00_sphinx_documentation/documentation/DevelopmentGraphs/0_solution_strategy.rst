Solution Strategy
=================

RoboyVR consists of different components which work together.
One big part deals with the transition
between the different coordinate frames of Gazebo and Unity. At first the rotations were
represented via Euler Angles, this lead to gimbal locks. To avoid this we switched to quaternions.
Roboy's pose needs to be converted to Unity's coordinate frame.
In addition we convert the model of roboy to a unity friendly format. 
The other part deals with user interaction. RoboyVR uses user input to manipulate the simulation
and renders the result on a GUI.




.. figure:: images/board_full.*
   :align: center
   :alt: Project whiteboard
      
   Whiteboard showing problems and solutions that occured during development of roboyVR.

   
.. figure:: images/view_comparison.*
   :alt: Comparison view sketch
   
   Handdrawn sketch showcasing the design of a specific UI Panelmode (comparison).
   
.. figure:: images/view_single.*
   :alt: Single view sketch
   
   Handdrawn sketch showcasing the design of a specific UI Panelmode (single).