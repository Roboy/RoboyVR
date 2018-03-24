User's Manual
=============

Starting the Simulation
-----------------------

On the computer running the Gazebo simulation, set up all required exports and so forth as described in the *Getting started* section. 

Run the launch file, which starts a ROS server and the simulation

.. code:: bash

  roslaunch roboy_simulation roboy_moveable.launch  

Additionally, the applied forces can be visualised in RViz to help with debugging and to gain further insight. Run the following command

.. code:: bash

  rviz
  
Make sure to follow the additional steps described in *Getting started* to be able to see Roboy and the applied force. 

Starting Unity
--------------
1. Start the unity project and set up the ROS server connection as described in **Getting Started**. 
2. Start the scene.
3. After the controller assignment, switch to the HandTool
4. Point at Roboy and grab him using the trigger
5. Move Roboy around

.. figure:: images/moving.*
    :align: center
    :alt: Roboy being moved
    
    Roboy being shoved around 
