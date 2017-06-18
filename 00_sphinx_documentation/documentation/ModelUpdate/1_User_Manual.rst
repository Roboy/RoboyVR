User's Manual
=============


.. figure:: images/UnityScene.*
   :target: https://www.youtube.com/watch?v=nNV-3x-7Jho
   :align: center
   :alt: https://www.youtube.com/watch?v=nNV-3x-7Jho
         
   Video showing the MeshUpdater


Part 1: Getting started
-----------------------

Open the Unity project RoboyVR. Open the RoboyViveScene, and select the RoboyMananger in the hierarchy tab.
The RoboyManager has a script called Mesh Updater. The following instructions all are entered here.

.. figure:: images/RoboyManager.*
   :align: center
   :alt: RoboyManager GUI
         
   Mesh Updater GUI


Part 2: Github Repository
-------------------------

Enter the link of the Github Repository where the models are located, which you want to download.
Make sure the link ends with a slash.

Default:
Github_Repository = https://github.com/Roboy/roboy_models/


Part 3: Set Blender.exe
-----------------------

Click on "Open Blender directory" and choose the blender.exe.

i.e.: C:\\Program Files\\Blender Foundation\\Blender\\blender.exe


Part 4: Scanning
----------------

Click "Scan" and wait until UnityEditor shows you every model in the Github_Repository.


Part 5: Downloading
-------------------

Select the models you want save as prefab and press "Download". You can select more than one model.
This may take a while, since the downloaded models will also automatically be imported into Unity.


Part 6: Create the Prefab
-------------------------

After importing the files, press "Create Prefab".
You can now find the created prefab in Assets\/SimulationModels\/...


