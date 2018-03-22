User's Manual
=============

**NOTICE: THIS CODE IS NOT BEING MAINTAINED AND MAY THEREFORE NOT WORK CORRECTLY ANYMORE OR THE INFORMATION MAY BE OUTDATED**

This manual will descripe the steps required to start BeRoboy.

Starting Gazebo
---------------
Start the Ubuntu machine and open a terminal. See further information  about starting a simulation under the point **Getting Started** - this includes necessary exports before a launch file is executed. 

Start the launch file which starts Gazebo with the Roboy and a Camera ROS node

.. code:: bash

  roslaunch roboy_simulation roboy_camera.launch  

If yout want the insert/remove feature to work, also launch this in a seperate terminal, calling all pre-simulation commands again.

.. code:: bash

  rosrun roboy_simulation VRRoboy  


Starting Unity
--------------
1. Start the unity project and set up the ROS server connection as described in **Getting Started**. 
2. Start the scene.
3. After the controller assignment, you can switch between various view modes via a selection menu in the scene.

.. figure:: images/view_selection_pointing.*
    :align: center
    :alt: View Selection

4. Enjoy!


View Scenarios
--------------
You can choose between the following four view scenarios, each of them offering different things to explore!

**I. Gazebo Simulation**

.. figure:: images/beroboy_simulation.*
   :align: center
   :alt: Simulation view
         
   Take control over the simulation Roboy and see what he does in gazebo.

**II. Real Roboy (ZED)**
 
.. figure:: images/beroboy_z3d.*
   :align: center
   :alt: ZED view
         
   Look through the eyes of the real Roboy and control him in real life.

**III. Observing Gentleman**
   
.. figure:: images/beroboy_observer.*
   :align: center
   :alt: Observer view
         
   Sit back, relax, take a look at Roboy from a safe distance and watch him do some stuff.
   
**IV. VR Roboy**
   
.. figure:: images/beroboy_vr.*
   :align: center
   :alt: VR view
         
   Slip into the role of the true VR Roboy, cause mayhem or look cute, you decide.


