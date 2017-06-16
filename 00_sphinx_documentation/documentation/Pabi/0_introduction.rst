Introduction
============

**What is it?**

PaBi-VR shows the Roboy PaBi legs in a simulation and in a VR room simultaneously while the PaBi legs shows some furious dance moves appropriate for every music track. On top of that you can stop the *EPIC* dance and create your own dance moves with simple commands.
In VR you can fully immerse yourself with the PaBi World, a world which you will not want to leave ever again. A tremendous GUI
visualizes *very interesting* data.

**How does it work?**
  
The PaBi legs are loaded in Gazebo via a Plugin which makes them listen to dance commands. At the same time a ROS node starts which sends dance commands to PaBi. The whole PaBi state is send to the VR room in Unity and is processed on a GUI.
