Introduction
============

**NOTICE: THIS CODE IS NOT BEING MAINTAINED AND MAY THEREFORE NOT WORK CORRECTLY ANYMORE OR THE INFORMATION MAY BE OUTDATED**

What is it?
-----------
PaBi-VR shows the Roboy PaBi legs in a virtual environment, where the model moves according to values given by a simulation or the real-life legs. This way, simulated dance moves or the real movement of the legs can be captured and investigated further. Additionally, new dance moves can be created by shooting at the model with the toy gun.

How does it work?
-----------------
The PaBi legs are simulated in Gazebo using a launch-file and model plugins, which receive and process the given forces.  A force is created, when a body part is hit by a foam bullet. The force position is set at the point of impact, and the information is then sent to the simulation. This, in turn, moves the legs according to the force values. Another option to animate the legs is to use a plugin, which sends continuous commands to the simulation to be applied to the model.  The resulting pose is then communicated to Unity and the VR model, where the new position is updated accordingly. 
