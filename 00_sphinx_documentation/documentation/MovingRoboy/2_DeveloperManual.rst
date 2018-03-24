Developer's Manual
==================

Gazebo Simulation
-----------------

For the gazebo part a launch/ world file automatically loads the world (with all the surrounding objects) that has been specified and the version of Roboy which was chosen. A plugin is in charge of applying the forces and sending the forces to be visualized by RViz. 

The simulation VRRoboy2 also applies the external forces and simulates Roboy, but due to a Gazebo bug, model plugins are not loaded. Using the .launch file is recommended.  

Model Configuration
-------------------
A new model roboy_simplified_moveable was derived from an existing model and additional values / changes were applied: 

- Roboy's feet are static, therefore glued to the ground
- Roboy is not affected by gravity
- The joints have a high friction: This, together with the strong forces applied, makes the inertia of Roboy's body neglectable / less prominent and he moves immediately 

Unity Scene
-----------
The forces which are to be applied are calculated in the HandTool and published over ROS. A position subscriber receives the updated position and applies it to the model. 
**Issue:** Unity works with around 80 FPS, while the simulation publishes around 300 messages per second. Even though the pose subscriber drops superfluous messges, a back-log of messages cannot be prevented on Unity-side as of now and the visualised model starts to lag behind. 

**Solution for now:** The simulation restricts the number of sent messages

Coordinate Systems
------------------
Coordinate systems presented a major issue in this project. Unity and gazebo have different coordinate systems, additionally, local and world coordinate systems add to the complexity. Please see the **Implementation/Coordinate Systems/** section for more information. 

.. figure:: images/coordinate_systems.*
   :align: center
   :alt: Wrong coordinate systems
         
   The result of local and world coordinate system discrepancies
   
