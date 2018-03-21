Introduction 
============

***NOTICE: LEGACY CODE. NOT FULLY FUNCTIONING: DOES NOT FIND MODEL FOLDERS IN REPOSITORY***

What is it?
-----------

The Automatic Update Pipeline is a convinient Unity Editor extension, which consists of two core features. 
The Model Updater is a tool to download new models or update existing ones which are used in the RoboyVR project. 
The World Updater downloads and updates worlds and environments created with gazebo.


How does it work?
-----------------

**Model Updater**

After a short setup, where the user has to select the blender.exe and set the models github_repo path, the user can scan the Github Repository for models.
Now a list of found models appears and the user can select, which specific models to download.
Pressing "Download" loads the models and additionally converts them with blender to .fbx, so they can be used in Unity. Additionally the model.sdf file is downloaded.
Because the models are downloaded into the Assets folder of the Unity project, they will automatically be imported into unity.
Afterwards you just have to press "Create Prefab" and the model will be saved as a prefab, the pose and scale of given in the model.sdf will be applied here. 
Finally the user can easily just drag and drop the model into the VR scene.

**World Updater**

Just like with the Model Updater first the blender.exe has to be selected and the world github_repo path hast to be set. 
Then the user can scan the Github Repository for worlds.
The list of found worlds appears and the user should select, which worlds to download.
After the user presses "Download", the .world file is downloaded and read. The .world file gives information, which models have to be downloaded for the environment.
Because the models are downloaded into the Assets folder of the Unity project, they will automatically be imported into unity.
Finally pressing "Create World" will save every model in the world as one prefab, which the user can easily just drag and drop into the VR scene.