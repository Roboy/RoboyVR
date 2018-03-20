Current State
=============

***NOTICE: LEGACY CODE. NOT FULLY FUNCTIONING: DOES NOT FIND MODEL FOLDERS IN REPOSITORY***

Right now the Updater is converting .dae and .stl files into .fbx to use them in unity. 
To set each model and link at the correct position, .sdf and .world files are read out and their poses are applied to the models/worlds.


Unfortunately the world_dom.py and sdf_dom.py need the .world/.sdf files in a certain format, 
so that most of these files have to be edited and saved in a sperate branch of the github repository.
For example the sdf_dom.py script can't handle "<pose frame=''>" or plugins.
As a result of this the MeshUpdater only works with arm_simplified in the VRTeam branch and the WorldUpdater only works with testworld2 in the roboy_worlds github repository.
Also as of right now, there is no texture applied to the downloaded models, so Unity just uses some standard ones, which might look bad.


In the future, we need to include textures in the Updater process. 
Also we should figure out if there is an easier way to use .sdf and .world files, without having to change the actual files.


