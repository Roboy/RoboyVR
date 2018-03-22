General Project Architecture
============================

Building Blocks View
--------------------

.. figure:: images/buildingblocks.*
   :alt: Bulding blocks overview

   RoboyVR Experience has several neighbouring systems like the simulation and github,
   it consists of various components like RoboyManager/Inputmanager and 
   can be manipulated by the user through the HMD system and the controllers.

Deployment View
---------------

.. figure:: images/deployment.*
  :alt: Roboy simulation runs on a virtual machine, RoboyVR Experience runs on Unity.
  
  Roboy simulation runs on Ubuntu on a seperate(virtual) machine, RoboyVR Experience runs on Unity.
  

**Note:**  Further information about certain implementation strategies can also be found in the *Developer's* Manual sub-sections of the individual projects *BeRoboy*, *State Visualisation*, *Model World Loader* and *PaBi*.