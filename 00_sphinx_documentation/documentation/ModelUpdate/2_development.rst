Developer's Manual
==================


Part 0: Summary
---------------


.. figure:: images/ModelUpdate.*
  :alt: Sequence Diagram


a fully automated model and world loading script

1. Model loading is controlled by simple GUI elements
2. Models as well as worlds are listed from the roboy_models and roboy_worlds repo for user selection
3. Selected models are downloaded and converted to use them in Unity 
4. The selected model and world (.dae or .stl meshes) are automatically saved in a prefab, which can easily be loaded into the scene and here enable the known interaction: selection of model parts and motor state visualization

	
Part 1: GUI elements
--------------------

MeshUpdaterEditor.cs: 
Custom editor script to be able to call functions from meshUpdater at edit time through buttons.
This is the GUI you use when updating a model. The GUI has different states, so the user can't skip necessary steps.

The first state is called "Initialized". 
In this state you can see the Github_Repository as a public string, used to find the models to download. 
You can put in any link, as long as the models are in the same folder hierarchy as in roboy_models.
Make sure the link ends with a slash.
Default string is "https://github.com/Roboy/roboy_models/"


If the blender directory isn't set, you can set it by clicking the button "Open Blender directory".
This will open the Windows Explorer and you have to select the "blender.exe".
After setting the blender directory, the state is changed to BlenderPathSet.
This state shows the blender path as a string. Here the is GUI disabled so it can't be edited in UnityEditor.


Also the state now shows a button called "Scan". This button calls meshUpdater.Scan() (`Part 2: Scanning`_).
When meshUpdater.Scan() finishes, the state will be changed to "Scanned".


In the new state, you can see which models were found by the scanning script and can select,
which of these you want to download by checking off the corresponding boxes.
The "Download" button then calls meshUpdater.UpdateModels() (`Part 3: Downloading`_), in which the state is set to "Downloaded".


After everything downloaded is imported in Unity, and state is set to "Downloaded", you can click the button "Create Prefab",
this will call meshUpdater.CreatePrefab() (`Part 4: Create Prefab`_).


Part 2: Scanning
----------------

meshUpdater.Scan():
First of all the scan function creates a local array scanArguments, filling it with {"python", m_PathToScanScript, Github_Repository}
This is used to RunCommandLine(scanArguments), which starts ModelScanner.py.

ModelScanner.py

Part 3: Downloading
-------------------

MeshUpdater.UpdateModels()
ModelDownloader.py

Part 4: Create Prefab
---------------------

MeshUpdater.CreatePrefab()