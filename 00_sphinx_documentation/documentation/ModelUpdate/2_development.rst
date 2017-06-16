Model Updater
=============


Part 0: What is it?
-------------------


.. figure:: images/ModelUpdate.*
  :alt: Sequence Diagram


a fully automated model and world loading script

1. Model loading is controlled by simple GUI elements
2. Models as well as worlds are listed from the roboy_models and roboy_worlds repo for user selection
3. Selected models are downloaded and converted to use them in Unity 
4. The selected model and world (.dae or .stl meshes) are automatically saved in a prefab, which can easily be loaded into the scene and here enable the known interaction: selection of model parts and motor state visualization

	
Part 1: GUI elements
--------------------

MeshUpdaterEditor:

Custom editor script to be able to call functions from MeshUpdater at edit time through buttons.
This is the GUI you use when updating a model.

Part 2: Scanning
----------------

MehsUpdater.Scan()
ModelScanner.py

Part 3: Downloading
-------------------

MeshUpdater.UpdateModels()
ModelDownloader.py

Part 4: Create Prefab
---------------------

MeshUpdater.CreatePrefab()