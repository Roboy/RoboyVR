Introduction
============

.. figure:: images/grab3.*
   :align: center
   :alt: Moving Roboy
         
   Moving Roboy in VR using the Hand Tool
   

What is it?
-----------

Roboy can be moved around in the virtual environment. The user can grab him, pull and push Roboy and shove him around. Roboy adapts his position based on a physical simulation of his body and the applied forces. He is glued to the ground to prevent him from crumbling to the ground and flying around in the room.  


How does it work?
-----------------

As soon as a user grabs a body part of Roboy, forces are calculated. The further the user pulls or pushes, the higher is the calculated force. This force is sent to the simulation running in Gazebo, which applies the forces and sends an updated position back to Unity, which adapts Roboy's pose. 