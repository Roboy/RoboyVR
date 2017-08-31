Current State
=============

Up until now the project has come a long way, many tools have been implemented to play around
with Roboy and it's environment. The core features provide various functions for the user. One
of the main abilities of the VR experience is BeRoboy, where the user gets the chance to control
Roboy. As of now only virtual Roboys can be manipulated and controlled which are serving as subjects
in the gazebo simulation. In the future it is planned to control also the real counterparts of Roboy.

Besides controlling Roboy it is possible to insert and remove numerous models from the virtual world
simply by point and click. For this purpose we designed and implemented a seperate tool which you can
selected from the left hand controller (selectionwheel is activated by the `gripped <images/controller_layout.png>`_ button).
Just choose the model you want to insert and click on a free space on the ground. This model will be inserted
in the world of Unity, but in the simulation PaBi will not be inserted as well as the surprise model.
This is due to missing content in the gazebo model structure on the simulation side.

In addition the models get inserted but their poses are not updated by the simulation because of lacking
code in the .cpp file. Only the pose of one model might be refreshed correctly. Removing models does not
work with all of the models, the surprise model and PaBi will not work.

On top of this functionality it is possible to download models and worlds from their respective github-repositories.
For this process it was necessary to create scripts which would scan for models/worlds and use a mesh updater to get
their latest version (see also the Automatic Update Pipeline section).

Teleporting in the master scene (which represents all core features) is also a bit buggy until now, the navigation
mesh would need an update as we increased the scale on our VR cave. This was done to give the user more space to
insert additional models in the environment. Shooting only works with one model in the scene, e.g. the same one where
also the pose subscribing works.