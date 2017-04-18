Getting started
===============

Part 1: Run rosbridge and roboySimulation
-----------------------------------------

.. code:: bash

  roslaunch rosbridge_server rosbridge_websocket.launch
  rosrun roboy_simulation VRRoboy
  
Part 2: Open the project in Unity
---------------------------------

Unity is organized in Scenes. In order to watch the simulation in Unity which is running on the VM (in gazebo),
open the ViveScene. 

.. image:: https://cloud.githubusercontent.com/assets/10234845/21025492/f72657fc-bd88-11e6-912e-877ba72d782e.png

Part 3: Setup the scene
-----------------------

In the Scene you can observe the simulation from the VM within Unity.
To do that you need to communicate the IP adress of your VM towards RoboyManager.
The IP information is quickly found in Ubuntu by clicking on the two arrows pointing in opposite directions,
right next to the system time. Afterwards a drop down menu will open, click on connection information.
Remember the IP and paste it in the respective field in Unity.

.. image:: https://cloud.githubusercontent.com/assets/10234845/21025737/da6cda5e-bd89-11e6-8755-af5fbf4a748b.png

You also need to drag the roboy prefab onto the RoboyManager if it is not already done. 
Each roboy model is tagged as a *RoboyPart*.
If you import new models for roboy you need to change the tag accordingly and change the roboy prefab.

.. image:: https://cloud.githubusercontent.com/assets/10234845/21025736/da6bcb1e-bd89-11e6-820d-be7b42853697.png

You can reset the simulation with the **R** key, you can also change the key in *RoboyManager*.
To get a better view of the simulation we recommend to set the simulation to slow motion in rviz in the VM:
  - If you want to start rviz, open a terminal (in the VM) and simply type **rviz**
  - Set Fixed Frame to World (Displays->Fixed Frame)
  - Add a marker (Add(Button)->marker)
  - Add walking plugin (Panels->Add New Panel->WalkingPlugin)
  - Turn slow motion on (within the walking plugin, it is a toggle button)
  
Extra: Update roboy models
--------------------------

*IMPORTANT: The next part will be soon outdated as we plan update the models and automate the process,
so be aware!*

In /roboyVR/Assets/RoboyModel/OriginModels there is a script **meshDownloadScript.py**. 
When executed, it downloads roboy models from this location:  
  - https://github.com/Roboy/roboy_models/tree/master/legs_with_upper_body/cad.  

After the download process is complete the models will be converted by blender so that they work fine with unity (.fbx format).
Obviously you need blender and python installed on your system so that the script can do it's work.
You can use the template **runScript** bat file for Windows. 

The format:

.. code:: batch

  start "" "pathToBlender/blender.exe" -P "pathToScript\meshDownloadScript.py" **meshes name seperated by ',' without whitespace and file format**

Example:

.. code:: batch

  start "" "C:\Programs\Blender\blender.exe" -P "C:\Documents\roboyVR\Assets\RoboyModel\OriginModels\meshDownloadScript.py" hip,torso,thigh_left,head

