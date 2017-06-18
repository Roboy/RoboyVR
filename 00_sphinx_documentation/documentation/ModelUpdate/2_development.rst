Developer's Manual
==================


Summary
-------


.. figure:: images/ModelUpdate.*
   :align: center
   :alt: Sequence Diagram
         
   Sequence Diagram for MeshUpdater



Prototype:
a fully automated model loading script

1. Model loading is controlled by simple GUI elements
2. Models are listed from the roboy_models repo for user selection
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


In the new state, you can see a list with the models that were found by the scanning script and can select,
which of these you want to download by checking off the corresponding boxes.
The "Download" button then calls meshUpdater.UpdateModels() (`Part 3: Downloading`_), in which the state is set to "Downloaded".


After every downloaded model is imported in Unity, and state is set to "Downloaded", you can click the button "Create Prefab",
this will call meshUpdater.CreatePrefab() (`Part 4: Create Prefab`_).


Part 2: Scanning
----------------

meshUpdater.Scan():
First of all the scan function creates a local array scanArguments, filling it with {"python", m_PathToScanScript, Github_Repository}
This is used to RunCommandLine(scanArguments), which starts ModelScanner.py.


ModelScanner.py scans the source code of the Github_Repository for links to subfolders by using regular expressions.
The names and links of the subfolders (models) will be saved in a temporary file that we can read in later on.


Now the Scan() function creates a <string, string> dictionary. This is filled with model names and their links, which were saved in the temporary file. 
Then names are also written in a <string, bool> ModelChoiceDictionary, which is used for the selection in the UnityEditor.
Lastly the current state is set to "Scanned".


Part 3: Downloading
-------------------

MeshUpdater.UpdateModels():
For every entry in ModelChoiceDictionary that is true, the ModelScanner.py is used to get each subfolder. This is because of the way the hierarchy is currently set up in the roboy_models github repository.
Now every link to the subfolders is given to the ModelDownloader.py, together with the m_PathToBlender and the path to where to download the models.


ModelDownloader.py is again scanning the source code, but this time not for folders, but for files with the .dae or .stf extension.
Then it downloads every model to the given path, by creating new files and copying the raw content of the files stored in github.
Finally all downloaded models are imported into blender, converted to a .fbx file and exported. The original files are overridden. The conversion is necessary so we can use the models in Unity.

Part 4: Create Prefab
---------------------

MeshUpdater.CreatePrefab():
Creates a GameObject called modelParent. 
With importModelCoroutine(string path, System.Action<GameObject> callback) a converted .fbx model in the model folder is loaded in a temporary GameObject meshCopy.
The meshCopy is then attached as a child to modelParent. This happens for every model in the model folder.
 

Afterwards an empty prefab is created with the name modelname.prefab. The prefab's content is then replaced by modelParent. At last modelParent is deleted since we don't need it anymore. 
