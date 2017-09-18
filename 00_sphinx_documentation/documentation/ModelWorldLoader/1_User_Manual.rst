User's Manual
=============

MeshUpdater
-----------

.. figure:: images/UnityScene.*
   :target: https://www.youtube.com/watch?v=nNV-3x-7Jho
   :align: center
   :alt: https://www.youtube.com/watch?v=nNV-3x-7Jho
         
   Video showing the MeshUpdater in previous Version


**Getting started**

Open the Unity project RoboyVR. Open the ModelUpdaterScene, and select the Updater in the hierarchy tab.
The Updater has a script called Mesh Updater. The following instructions all are entered here.

.. figure:: images/UpdaterModel.*
   :align: center
   :alt: Updater MeshUpdater.cs GUI
         
   Mesh Updater GUI


**Github Repository**

Enter the link of the Github Repository where the models are located, which you want to download.
Make sure the link ends with a slash. Also you can set here, which branch you want to download the models from.

Default:

- Github_Repository = https://github.com/Roboy/roboy_models/

- Branch = VRTeam


**Set Blender.exe**

Click on "Open Blender directory" and choose the blender.exe.

i.e.: C:\\Program Files\\Blender Foundation\\Blender\\blender.exe


**Scanning**

Click "Scan" and wait until UnityEditor shows you every model in the Github_Repository.


**Downloading**

Select the models you want save as prefab and press "Download". You can select more than one model.
This may take a while, since the downloaded models will also automatically be imported into Unity.


**Create the Prefab**

After importing the files, press "Create Prefab".
You can now find the created prefab in Assets\/SimulationModels\/...


WorldUpdater
------------

**Getting started**

Open the Unity project RoboyVR. Open the ModelUpdaterScene, and select the Updater in the hierarchy tab.
The Updater has a script called World Updater. The following instructions all are entered here.


**Github Repository**

Enter the link of the Github Repository where the worlds and related models are located.
Make sure the link ends with a slash. Also set here, which branch is used to download the models from.

Default:

- Github_Repository = https://github.com/Roboy/roboy_worlds/

- Branch = master


**Set Blender.exe**

Click on "Open Blender directory" and choose the blender.exe.

i.e.: C:\\Program Files\\Blender Foundation\\Blender\\blender.exe


**Scanning**

Click "Scan" and wait until UnityEditor shows every world located in the Github_Repository.


**Downloading**

Select the worlds you want save as prefab and press "Download". You can select more than one world.
This may take a while, since the downloaded worlds and related models will also automatically be imported into Unity.


**Create the World**

After importing the files, press "Create World".
You can now find the created world in Assets\/SimulationWorlds\/...


Troubleshooting
---------------

Please take a look at the current state chapter.


**State not set correctly**

If the Editor looks like this, without a blender path set: 

.. figure:: images/Error1.*
   :align: center
   :alt: MeshUpdaterEditor state not correct
         
   state not correct

Try removing the script and adding it again.

.. figure:: images/Solution1.*
   :align: center
   :alt: Removing script
         
   Removing the Updater script

.. figure:: images/Solution1,1.*
   :align: center
   :alt: Adding script
         
   Reloading the Updater script